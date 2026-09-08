# haikalbaiqunni.github.io

Personal website of **Haikal Hakim Baiqunni** — R&D Engineer at Mitutoyo Corporation, Japan.
Mechatronics, precision non-contact measurement, smart sensors, motor control, and AI-augmented engineering.

**Live:** https://haikalbaiqunni.github.io

## Stack

Static HTML + CSS, no framework, no build step. One file does the work:

```
index.html                    # the whole site
404.html                      # styled not-found page
assets/favicon.svg            # drawing-crosshair mark
assets/Haikal_Baiqunni_CV.pdf # downloadable CV
robots.txt / sitemap.xml      # indexing
.nojekyll                     # serve files as-is on GitHub Pages
```

The design is a technical-drawing metaphor: blueprint grid, dimension callouts, a revision log
for the work history, a parts list for projects, and an engineering title block as the footer.

## Local preview

No dependencies needed — open `index.html` in a browser, or serve it:

```bash
python -m http.server 8000
```

Then visit http://localhost:8000

## Deployment

Pushing to `main` publishes automatically via GitHub Pages (Settings → Pages → Deploy from
branch → `main` / `/root`).

## License

Code is MIT licensed (see [LICENSE](LICENSE)). Written content, CV, and personal
information are not covered — please don't reuse those.
