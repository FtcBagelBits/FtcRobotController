package org.firstinspires.ftc.teamcode.pollen;

import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.Proxy;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.Semaphore;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/** A viewing-only HTTP gateway. No FTC/Android dependencies, camera writes, or motor access.
 * Network I/O stays on bounded background workers. Only one upstream video stream is allowed.
 * The OpMode owns this instance from INIT to STOP; it is never an app-wide service. */
public final class PollenCameraViewer implements Closeable {
    public static final int PORT = 5800;
    // Default USB index 0. This is the CAMERA's address, not the Hub's USB interface address.
    public static final String CAMERA_STREAM = "http://172.29.0.1:5800/";
    private static final String PAGE = "<!doctype html><html><head><meta charset='utf-8'>"
            + "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            + "<title>Pollen Camera Drive</title><style>body{background:#17212b;color:#fff;"
            + "font:18px system-ui;margin:24px}img{display:block;max-width:100%;margin-top:16px}"
            + "button{font:inherit;padding:8px;margin-right:12px}</style></head><body>"
            + "<h1>Pollen Camera Drive</h1><p>Live Limelight view. Drive with the gamepad, watching the robot.</p>"
            + "<p>L1: hold to drive (30%). L1 + R1: fine positioning (15%). Release L1 to stop.</p>"
            + "<button onclick='location.reload()'>Reconnect video</button>"
            + "<button onclick=\"document.getElementById('video').src='';this.disabled=true\">Pause video</button>"
            + "<p id='status'>Video can lag or freeze. Check freshness on the Driver Hub. STOP closes this page's server.</p>"
            + "<img id='video' src='/stream' alt='Limelight live stream' "
            + "onerror=\"document.getElementById('status').textContent='Stream unavailable. Check Driver Hub Viewer status, close other viewers, then reconnect.'\">"
            + "</body></html>";
    private final URL upstreamUrl;
    private final ServerSocket listener;
    private final ThreadPoolExecutor workers;
    private final Semaphore streamSlot = new Semaphore(1);
    private final Set<Socket> clients = new HashSet<>();
    private final AtomicLong bytes = new AtomicLong();
    private volatile boolean closed;
    private volatile HttpURLConnection upstream;
    private volatile String status = "Ready; open the viewer on robot Wi-Fi";

    public PollenCameraViewer() throws IOException { this(PORT, new URL(CAMERA_STREAM)); }

    /** Port 0 and a local fake camera are used by desktop tests. */
    PollenCameraViewer(int port, URL upstreamUrl) throws IOException {
        this.upstreamUrl = upstreamUrl;
        listener = new ServerSocket();
        try {
            listener.setReuseAddress(true);
            listener.bind(new InetSocketAddress(port));
        } catch (IOException ex) { listener.close(); throw ex; }
        workers = new ThreadPoolExecutor(0, 3, 10, TimeUnit.SECONDS, new SynchronousQueue<>(), runnable -> {
            Thread thread = new Thread(runnable, "Pollen-camera-client");
            thread.setDaemon(true);
            return thread;
        });
        Thread acceptor = new Thread(this::accept, "Pollen-camera-listener");
        acceptor.setDaemon(true);
        acceptor.start();
    }

    public int port() { return listener.getLocalPort(); }
    public String status() { return status; }
    public long bytes() { return bytes.get(); }

    private void accept() {
        while (!closed) {
            try {
                Socket client = listener.accept();
                synchronized (clients) {
                    if (closed) { quietly(client); return; }
                    clients.add(client);
                }
                try { workers.execute(() -> serve(client)); }
                catch (RejectedExecutionException ex) { forget(client); }
            } catch (IOException ex) {
                if (!closed) { status = "Listener failed: " + ex.getMessage(); close(); }
            }
        }
    }

    private void serve(Socket client) {
        try {
            client.setSoTimeout(3000);
            String[] request = headers(client.getInputStream()).split("\r\n", 2)[0].split(" ");
            OutputStream out = client.getOutputStream();
            if (request.length != 3 || !request[0].equals("GET")) {
                text(out, "405 Method Not Allowed", "text/plain", "GET only");
            } else if (request[1].equals("/")) {
                text(out, "200 OK", "text/html; charset=utf-8", PAGE);
            } else if (request[1].equals("/stream")) {
                stream(out);
            } else {
                text(out, "404 Not Found", "text/plain", "Only / and /stream are available");
            }
        } catch (IOException ignored) {
            // Browser disconnects / incomplete requests do not affect the robot or listener.
        } finally { forget(client); }
    }

    private void stream(OutputStream out) throws IOException {
        if (!streamSlot.tryAcquire()) {
            text(out, "503 Service Unavailable", "text/plain", "Close the other video viewer first");
            return;
        }
        HttpURLConnection connection = null;
        boolean sentHeaders = false;
        try {
            connection = (HttpURLConnection) upstreamUrl.openConnection(Proxy.NO_PROXY);
            upstream = connection;
            if (closed) return;
            connection.setConnectTimeout(2000);
            connection.setReadTimeout(3000);
            connection.setInstanceFollowRedirects(false);
            connection.setUseCaches(false);
            int code = connection.getResponseCode();
            String type = connection.getContentType();
            if (code != 200 || type == null || !(type.startsWith("multipart/x-mixed-replace")
                    || type.startsWith("image/")) || type.contains("\r") || type.contains("\n"))
                throw new IOException("Camera stream returned " + code + " / " + type);
            status = "Streaming";
            out.write(response("200 OK", type).getBytes(StandardCharsets.US_ASCII));
            out.flush();
            sentHeaders = true;
            try (InputStream in = connection.getInputStream()) {
                byte[] buffer = new byte[16384];
                int count;
                while (!closed && (count = in.read(buffer)) != -1) {
                    out.write(buffer, 0, count);
                    out.flush();
                    bytes.addAndGet(count);
                }
            }
            status = "Stream ended; reconnect in the browser";
        } catch (IOException ex) {
            status = "Video ended/unavailable: " + ex.getMessage();
            if (!sentHeaders && !closed) text(out, "502 Bad Gateway", "text/plain", status);
        } finally {
            if (connection != null) connection.disconnect();
            upstream = null;
            streamSlot.release();
        }
    }

    private static String headers(InputStream in) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        int tail = 0;
        while (bytes.size() < 8192) {
            int b = in.read();
            if (b < 0) throw new IOException("Incomplete request");
            bytes.write(b); tail = (tail << 8) | b;
            if (tail == 0x0d0a0d0a) return new String(bytes.toByteArray(), StandardCharsets.US_ASCII);
        }
        throw new IOException("Request too large");
    }

    private static String response(String code, String type) {
        return "HTTP/1.1 " + code + "\r\nContent-Type: " + type
                + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    }

    private static void text(OutputStream out, String code, String type, String body) throws IOException {
        out.write(response(code, type).getBytes(StandardCharsets.US_ASCII));
        out.write(body.getBytes(StandardCharsets.UTF_8));
        out.flush();
    }

    private void forget(Socket client) {
        quietly(client);
        synchronized (clients) { clients.remove(client); }
    }

    @Override public void close() {
        closed = true;
        quietly(listener);
        synchronized (clients) {
            for (Socket client : clients) quietly(client);
            clients.clear();
        }
        // Close the browser sockets before interrupting workers, including a blocked write.
        workers.shutdownNow();
        HttpURLConnection connection = upstream;
        if (connection != null) connection.disconnect();
    }

    private static void quietly(Closeable resource) {
        try { resource.close(); } catch (IOException ignored) { }
    }
}
