// Robot Repository page: file counts on the section links + a full-size photo viewer.
(() => {
  // Show how many items each section holds next to its jump link.
  document.querySelectorAll("[data-count-for]").forEach((badge) => {
    const section = document.getElementById(badge.dataset.countFor);
    const count = section ? section.querySelectorAll("[data-repo-item]").length : 0;
    badge.textContent = count ? String(count) : "";
    badge.hidden = !count;
  });

  const links = Array.from(document.querySelectorAll(".repo-gallery a[data-lightbox]"));
  if (!links.length) return;

  const icon = (d) =>
    `<svg viewBox="0 0 24 24" aria-hidden="true"><path d="${d}" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/></svg>`;

  const box = document.createElement("div");
  box.className = "repo-lightbox";
  box.hidden = true;
  box.setAttribute("role", "dialog");
  box.setAttribute("aria-modal", "true");
  box.setAttribute("aria-label", "Photo viewer");
  box.innerHTML = `
    <button class="repo-lb-btn repo-lb-close" type="button" aria-label="Close photo viewer">${icon("M6 6l12 12M18 6L6 18")}</button>
    <button class="repo-lb-btn repo-lb-prev" type="button" aria-label="Previous photo">${icon("M15 5l-7 7 7 7")}</button>
    <figure class="repo-lb-figure">
      <img class="repo-lb-img" alt="">
      <figcaption class="repo-lb-caption"><span class="repo-lb-text"></span><span class="repo-lb-count"></span></figcaption>
    </figure>
    <button class="repo-lb-btn repo-lb-next" type="button" aria-label="Next photo">${icon("M9 5l7 7-7 7")}</button>`;
  document.body.appendChild(box);

  const img = box.querySelector(".repo-lb-img");
  const text = box.querySelector(".repo-lb-text");
  const counter = box.querySelector(".repo-lb-count");
  const closeBtn = box.querySelector(".repo-lb-close");
  const prevBtn = box.querySelector(".repo-lb-prev");
  const nextBtn = box.querySelector(".repo-lb-next");
  const single = links.length < 2;
  prevBtn.hidden = single;
  nextBtn.hidden = single;
  counter.hidden = single;

  let current = 0;
  let returnFocus = null;

  function show(index) {
    current = (index + links.length) % links.length;
    const link = links[current];
    const thumb = link.querySelector("img");
    const caption = link.closest("figure") && link.closest("figure").querySelector("figcaption");
    img.src = link.href;
    img.alt = thumb ? thumb.alt : "";
    text.textContent = caption ? caption.textContent.trim() : "";
    counter.textContent = `${current + 1} / ${links.length}`;
    if (!single) new Image().src = links[(current + 1) % links.length].href; // preload the next photo
  }

  function open(index) {
    returnFocus = document.activeElement;
    show(index);
    box.hidden = false;
    document.documentElement.classList.add("repo-lb-open");
    closeBtn.focus();
  }

  function close() {
    box.hidden = true;
    document.documentElement.classList.remove("repo-lb-open");
    img.removeAttribute("src");
    if (returnFocus) returnFocus.focus();
  }

  links.forEach((link, index) => {
    link.addEventListener("click", (event) => {
      if (event.button !== 0 || event.metaKey || event.ctrlKey || event.shiftKey || event.altKey) return;
      event.preventDefault();
      open(index);
    });
  });

  closeBtn.addEventListener("click", close);
  prevBtn.addEventListener("click", () => show(current - 1));
  nextBtn.addEventListener("click", () => show(current + 1));
  box.addEventListener("click", (event) => {
    if (event.target === box || event.target.classList.contains("repo-lb-figure")) close();
  });

  document.addEventListener("keydown", (event) => {
    if (box.hidden) return;
    if (event.key === "Escape") {
      close();
    } else if (event.key === "ArrowLeft" && !single) {
      show(current - 1);
    } else if (event.key === "ArrowRight" && !single) {
      show(current + 1);
    } else if (event.key === "Tab") {
      // Keep keyboard focus inside the viewer while it is open.
      const focusable = [closeBtn, prevBtn, nextBtn].filter((button) => !button.hidden);
      const at = focusable.indexOf(document.activeElement);
      event.preventDefault();
      const next = event.shiftKey ? (at <= 0 ? focusable.length - 1 : at - 1) : (at + 1) % focusable.length;
      focusable[next].focus();
    }
  });

  // Swipe left/right on touch screens.
  let touchX = null;
  box.addEventListener("touchstart", (event) => { touchX = event.touches[0].clientX; }, { passive: true });
  box.addEventListener("touchend", (event) => {
    if (touchX === null || single) return;
    const dx = event.changedTouches[0].clientX - touchX;
    touchX = null;
    if (Math.abs(dx) > 50) show(current + (dx < 0 ? 1 : -1));
  });
})();
