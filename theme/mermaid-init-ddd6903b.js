/* Render Mermaid if the library is present; allow CDN fallback if needed. */
(function () {
  function initMermaid() {
    if (!window.mermaid) {
      return;
    }
    window.mermaid.initialize({ startOnLoad: true });
  }

  if (window.mermaid) {
    initMermaid();
    return;
  }

  var script = document.createElement("script");
  script.src = "https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js";
  script.onload = initMermaid;
  document.head.appendChild(script);
})();
