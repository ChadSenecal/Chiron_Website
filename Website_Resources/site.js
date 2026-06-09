const boinkBtn = document.getElementById("boinkBtn");
const hamburger = document.getElementById("hamburger");
const mobileMenu = document.getElementById("mobileMenu");

const soundPresets = {
  documentation: {
    type: "triangle",
    notes: [330, 392, 495],
    duration: 0.26,
    volume: 0.08,
  },
  robot: {
    type: "square",
    notes: [196, 246.94, 329.63],
    duration: 0.22,
    volume: 0.055,
  },
  sponsors: {
    type: "sine",
    notes: [523.25, 659.25, 783.99],
    duration: 0.28,
    volume: 0.075,
  },
  team: {
    type: "triangle",
    notes: [440, 554.37, 659.25],
    duration: 0.24,
    volume: 0.075,
  },
  default: {
    type: "sine",
    notes: [392, 523.25],
    duration: 0.22,
    volume: 0.07,
  },
};
const soundCycle = ["documentation", "robot", "sponsors", "team", "default"];
let soundIndex = 0;

function playBoink(soundName = "default") {
  const ctx = new (window.AudioContext || window.webkitAudioContext)();
  const now = ctx.currentTime;
  const preset = soundPresets[soundName] || soundPresets.default;
  const step = preset.duration / preset.notes.length;

  const osc = ctx.createOscillator();
  const gain = ctx.createGain();
  osc.connect(gain);
  gain.connect(ctx.destination);
  osc.type = preset.type;
  preset.notes.forEach((note, index) => {
    const at = now + index * step;
    if (index === 0) {
      osc.frequency.setValueAtTime(note, at);
    } else {
      osc.frequency.exponentialRampToValueAtTime(note, at);
    }
  });
  gain.gain.setValueAtTime(0.001, now);
  gain.gain.exponentialRampToValueAtTime(preset.volume, now + 0.018);
  gain.gain.exponentialRampToValueAtTime(0.001, now + preset.duration);
  osc.start(now);
  osc.stop(now + preset.duration + 0.03);
}

if (boinkBtn) {
  boinkBtn.addEventListener("click", () => {
    const soundName = soundCycle[soundIndex % soundCycle.length];
    soundIndex += 1;
    playBoink(soundName);
    boinkBtn.classList.remove("boinking");
    void boinkBtn.offsetWidth;
    boinkBtn.classList.add("boinking");
    boinkBtn.addEventListener("animationend", () => boinkBtn.classList.remove("boinking"), { once: true });
  });
}

if (hamburger && mobileMenu) {
  hamburger.addEventListener("click", () => {
    const isOpen = mobileMenu.classList.toggle("open");
    hamburger.setAttribute("aria-expanded", String(isOpen));
  });

  mobileMenu.querySelectorAll("a").forEach((link) => {
    link.addEventListener("click", () => {
      mobileMenu.classList.remove("open");
      hamburger.setAttribute("aria-expanded", "false");
    });
  });
}
