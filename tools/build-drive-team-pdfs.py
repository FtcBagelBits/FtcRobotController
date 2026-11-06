#!/usr/bin/env python3
"""Build the printable drive-team PDFs in docs/drive-team/print/ from the Markdown pages.
Usage: python3 tools/build-drive-team-pdfs.py
Needs: pip install markdown playwright && python3 -m playwright install chromium
Edit the Markdown, not the PDFs. Redraw pictures first with tools/draw-drive-team-figures.py."""
import os
import pathlib
import markdown
from playwright.sync_api import sync_playwright

ROOT = pathlib.Path(__file__).resolve().parent.parent
PAGES = ROOT / 'docs' / 'drive-team'
OUT = PAGES / 'print'
SHEETS = ['1-shoot-and-park', '2-supporting-shooter', '3-teleop',
          'fallback-timed-autos', 'fallback-leave-the-wall', 'fallback-park-only']

CSS = """
@page { size: Letter; margin: 0.45in 0.55in; }
body { font-family: -apple-system, 'Helvetica Neue', Arial, sans-serif; font-size: 11pt; line-height: 1.3; color: #111; }
h1 { font-size: 20pt; margin: 0 0 4pt; border-bottom: 3px solid #1f6fb2; padding-bottom: 3pt; }
h2 { font-size: 13.5pt; margin: 8pt 0 3pt; color: #1f4f80; break-after: avoid; }
p { margin: 4pt 0; }
ul { margin: 3pt 0 3pt 0; padding-left: 18pt; }
li { margin: 1.5pt 0; }
table { border-collapse: collapse; width: 100%; margin: 3pt 0 5pt; font-size: 10pt; break-inside: avoid; }
th, td { border: 1px solid #999; padding: 2pt 5pt; text-align: left; vertical-align: top; }
th { background: #e9eef5; }
.side td:first-child { white-space: nowrap; }
code { font-family: Menlo, Consolas, monospace; font-size: 9.5pt; background: #f1f1f1; padding: 0 2pt; border-radius: 2px; }
img { display: block; margin: 4pt auto; break-inside: avoid; }
/* Pictures sit beside the text they explain, so a guide fits on one sheet (front and back). */
/* Picture beside its text (div.side), and a page break between front and back, so each main
   guide prints on one sheet. */
.side { display: grid; grid-template-columns: 3.35in 1fr; gap: 14pt; align-items: start; margin: 4pt 0; }
.side img { width: 100%; margin: 0; }
.side.buttons { grid-template-columns: 3.6in 1fr; }
.page-break { break-after: page; }
img[src*="controller"] { width: 7.3in; }
a { color: inherit; text-decoration: none; }
.footer { margin-top: 6pt; font-size: 8pt; color: #666; border-top: 1px solid #ccc; padding-top: 2pt; }
"""


def html_for(name):
    text = (PAGES / f'{name}.md').read_text()
    body = markdown.markdown(text, extensions=['tables', 'md_in_html'])
    base = PAGES.as_uri() + '/'
    return (f'<!DOCTYPE html><html><head><meta charset="utf-8"><base href="{base}"><style>{CSS}</style></head>'
            f'<body>{body}<div class="footer">Team 31452 drive team guide - build pollen-odo-v5 - '
            f'source: docs/drive-team/{name}.md</div></body></html>')


def main():
    OUT.mkdir(exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page()
        for name in SHEETS:
            tmp = PAGES / f'.{name}.html'  # beside the page so relative image paths resolve
            tmp.write_text(html_for(name))
            try:
                page.goto(tmp.as_uri())
                page.wait_for_load_state('networkidle')
                page.pdf(path=str(OUT / f'{name}.pdf'), format='Letter', print_background=True, prefer_css_page_size=True)
            finally:
                tmp.unlink()
        pit = (ROOT / 'docs' / 'POLLEN_PIT_SHEET.svg').as_uri()
        page.set_content('<!DOCTYPE html><html><head><style>@page { size: Letter landscape; margin: 0.25in; }'
                         'body { margin: 0; } img { display: block; margin: 0 auto; max-width: 100%; max-height: 7.9in; }</style></head>'
                         f'<body><img src="{pit}"></body></html>')
        page.wait_for_load_state('networkidle')
        page.pdf(path=str(OUT / 'pit-sheet.pdf'), print_background=True, prefer_css_page_size=True)
        browser.close()
    for f in sorted(OUT.glob('*.pdf')):
        print(f.relative_to(ROOT), f.stat().st_size // 1024, 'KB')


if __name__ == '__main__':
    main()
