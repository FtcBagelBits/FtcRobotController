package org.firstinspires.ftc.teamcode.pollen;

import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/** Real loopback sockets and a fake MJPEG camera. No robot, Android or outside network. */
public final class PollenCameraViewerTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static String head(InputStream in) throws IOException {
        ByteArrayOutputStream result = new ByteArrayOutputStream();
        int tail = 0;
        while (result.size() < 8192) {
            int b = in.read();
            if (b < 0) throw new IOException("EOF before headers");
            result.write(b); tail = (tail << 8) | b;
            if (tail == 0x0d0a0d0a) return new String(result.toByteArray(), StandardCharsets.US_ASCII);
        }
        throw new IOException("Header too long");
    }
    private static String body(InputStream in) throws IOException {
        ByteArrayOutputStream result = new ByteArrayOutputStream();
        byte[] buffer = new byte[1024]; int n;
        while ((n = in.read(buffer)) >= 0) result.write(buffer, 0, n);
        return new String(result.toByteArray(), StandardCharsets.UTF_8);
    }
    private static Socket request(int port, String method, String path) throws IOException {
        Socket socket = new Socket("127.0.0.1", port);
        socket.setSoTimeout(5000);
        socket.getOutputStream().write((method + " " + path + " HTTP/1.1\r\nHost: localhost\r\n\r\n")
                .getBytes(StandardCharsets.US_ASCII));
        return socket;
    }
    private static final class FakeCamera implements Closeable {
        final ServerSocket server = new ServerSocket(0);
        final AtomicInteger requests = new AtomicInteger();
        final CountDownLatch streaming = new CountDownLatch(1), release = new CountDownLatch(1);
        volatile String mode = "image";
        volatile Socket active;
        FakeCamera() throws IOException {
            Thread thread = new Thread(() -> {
                while (!server.isClosed()) {
                    try (Socket socket = server.accept()) {
                        active = socket;
                        socket.setSoTimeout(3000);
                        head(socket.getInputStream()); requests.incrementAndGet();
                        OutputStream out = socket.getOutputStream();
                        if (mode.equals("error")) out.write("HTTP/1.1 500 Error\r\nContent-Length: 0\r\n\r\n".getBytes(StandardCharsets.US_ASCII));
                        else if (mode.equals("stall")) { streaming.countDown(); release.await(5, TimeUnit.SECONDS); }
                        else if (mode.equals("stream")) {
                            out.write(("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
                                    + "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\nJPEG\r\n").getBytes(StandardCharsets.US_ASCII));
                            out.flush(); streaming.countDown(); release.await(5, TimeUnit.SECONDS);
                        } else {
                            out.write("HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\n".getBytes(StandardCharsets.US_ASCII));
                            out.write(new byte[]{(byte) 0xff, (byte) 0xd8, (byte) 0xff, (byte) 0xd9});
                        }
                        out.flush();
                    } catch (Exception ignored) { }
                    finally { active = null; }
                }
            }, "Fake-camera");
            thread.setDaemon(true); thread.start();
        }
        URL url() throws IOException { return new URL("http://127.0.0.1:" + server.getLocalPort() + "/"); }
        @Override public void close() throws IOException {
            release.countDown(); server.close();
            Socket socket = active; if (socket != null) socket.close();
        }
    }

    public static void main(String[] args) throws Exception {
        try (FakeCamera camera = new FakeCamera(); PollenCameraViewer viewer = new PollenCameraViewer(0, camera.url())) {
            try (Socket page = request(viewer.port(), "GET", "/")) {
                check(head(page.getInputStream()).contains("200 OK"), "page served");
                check(body(page.getInputStream()).contains("src='/stream'"), "page uses local video endpoint");
                check(camera.requests.get() == 0, "no camera request until video is requested");
            }
            for (String path : new String[]{"/pipeline-switch", "/stream?url=http://example.com", "/status"}) {
                try (Socket denied = request(viewer.port(), "GET", path)) {
                    check(head(denied.getInputStream()).contains("404"), "unrecognized route rejected");
                }
            }
            try (Socket denied = request(viewer.port(), "POST", "/stream")) {
                check(head(denied.getInputStream()).contains("405"), "writes rejected");
            }
            check(camera.requests.get() == 0, "rejected requests never reach camera");
            HttpURLConnection image = (HttpURLConnection) new URL("http://127.0.0.1:" + viewer.port() + "/stream").openConnection();
            image.setReadTimeout(5000);
            check(image.getResponseCode() == 200, "image response forwarded");
            try (InputStream in = image.getInputStream()) {
                check(in.read() == 255 && in.read() == 216 && in.read() == 255 && in.read() == 217 && in.read() == -1,
                        "binary JPEG bytes preserved");
            } finally { image.disconnect(); }
            check(viewer.bytes() == 4, "transfer counter");
            camera.mode = "error";
            try (Socket failed = request(viewer.port(), "GET", "/stream")) {
                check(head(failed.getInputStream()).contains("502"), "upstream failure reported");
                body(failed.getInputStream());
            }
            camera.mode = "image";
            try (Socket retry = request(viewer.port(), "GET", "/stream")) {
                check(head(retry.getInputStream()).contains("200"), "reconnect works after failure");
                body(retry.getInputStream());
            }
        }
        try (FakeCamera camera = new FakeCamera(); PollenCameraViewer viewer = new PollenCameraViewer(0, camera.url())) {
            camera.mode = "stream";
            int port = viewer.port();
            try (Socket stream = request(port, "GET", "/stream")) {
                check(head(stream.getInputStream()).contains("boundary=frame"), "MJPEG boundary preserved");
                check(camera.streaming.await(2, TimeUnit.SECONDS), "upstream streaming");
                try (Socket second = request(port, "GET", "/stream")) {
                    check(head(second.getInputStream()).contains("503"), "second video viewer bounded");
                }
                long before = System.nanoTime();
                viewer.close();
                check((System.nanoTime() - before) / 1e6 < 500, "STOP must not wait on upstream stream");
                try (ServerSocket rebound = new ServerSocket(port)) { check(rebound.isBound(), "port reusable after STOP"); }
                body(stream.getInputStream()); // must finish, not hang on the live upstream
            }
        }
        try (FakeCamera camera = new FakeCamera(); PollenCameraViewer viewer = new PollenCameraViewer(0, camera.url())) {
            camera.mode = "stall";
            try (Socket stream = request(viewer.port(), "GET", "/stream")) {
                check(camera.streaming.await(2, TimeUnit.SECONDS), "camera hangs before headers");
                long before = System.nanoTime(); viewer.close();
                check((System.nanoTime() - before) / 1e6 < 500, "STOP bounded during stalled camera response");
                check(stream.getInputStream().read() == -1, "client closed on STOP");
            }
        }
        System.out.println("PASS PollenCameraViewerTest: " + checks + " checks");
    }
}
