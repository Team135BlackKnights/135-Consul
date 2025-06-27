import { NT4_Client } from "./utils/NT4.js";

const ntClient = new NT4_Client(
  window.location.hostname,
  "OperatorControlsGrid",
  () => { },
  () => { },
  (topic, _, value) => {
    if (topic.name === toDashboardPrefix + "TopRow") {
      rowState.Top = value;
    } else if (topic.name === toDashboardPrefix + "MidRow") {
      rowState.Mid = value;
    } else if (topic.name === toDashboardPrefix + "HybridRow") {
      rowState.Hybrid = value;
    }
    renderButtons();
  },
  () => console.log("Connected to NT"),
  () => console.log("Disconnected from NT")
);


// === Constants ===
const toRobotPrefix = "/OperatorControls/ToRobot/";
const toDashboardPrefix = "/OperatorControls/ToDashboard/";
const ROWS = ["Top", "Mid", "Hybrid"];
const targetBoxTopicName = "TargetBox";
const COLS = 9;

// === Bitfield State ===
let rowState = {
  Top: 0,
  Mid: 0,
  Hybrid: 0, // 2 bits per button (0: none, 1: cone, 2: cube)
};

// === Target Mode State ===
let targetMode = false;
let currentTarget = null; // string like "Hybrid Cone 1"

// Add toggle button to DOM on load
window.addEventListener("load", () => {
  // Create toggle button
  const toggleBtn = document.createElement("button");
  toggleBtn.id = "target-mode-toggle";
  toggleBtn.textContent = "Enter Target Mode";
  toggleBtn.style.position = "fixed";
  toggleBtn.style.top = "1vh";
  toggleBtn.style.right = "1vw";
  toggleBtn.style.zIndex = "1000";
  document.body.appendChild(toggleBtn);




  toggleBtn.onclick = () => {
    targetMode = !targetMode;
    toggleBtn.textContent = targetMode ? "Exit Target Mode" : "Enter Target Mode";
    if (!targetMode) {
      // Remove this line: clearTargetHighlights();
      currentTarget = null;  // Optional: remove this if you want to keep the current target
    }
  };
  ntClient.subscribe(
    [
      toDashboardPrefix + "TopRow",
      toDashboardPrefix + "MidRow",
      toDashboardPrefix + "HybridRow",
    ],
    false,
    false,
    0.02
  );
  ntClient.publishTopic(toRobotPrefix + targetBoxTopicName, "string");
  ntClient.publishTopic(toRobotPrefix + "TopRow", "int");
  ntClient.publishTopic(toRobotPrefix + "MidRow", "int");
  ntClient.publishTopic(toRobotPrefix + "HybridRow", "int");

  ntClient.connect();
  renderButtons();
});
// And similarly in the context menu handler
document.addEventListener('contextmenu', (e) => {
  e.preventDefault();
  targetMode = !targetMode;
  const toggleBtn = document.getElementById('target-mode-toggle');
  toggleBtn.textContent = targetMode ? "Exit Target Mode" : "Enter Target Mode";
});
function clearTargetHighlights() {
  document.querySelectorAll(".grid-button.targeted").forEach((el) => {
    el.classList.remove("targeted");
  });
}

function renderButtons() {
  ROWS.forEach((rowName) => {
    const rowEl = document.getElementById(`${rowName.toLowerCase()}-row`);
    rowEl.innerHTML = "";

    for (let i = 0; i < COLS; i++) {
      const btn = document.createElement("div");
      btn.classList.add("grid-button");

      // Style by bitfield value
      if (rowName === "Hybrid") {
        const val = (rowState.Hybrid >> (i * 2)) & 0b11;
        if (val === 1) btn.classList.add("Algae");
        else if (val === 2) btn.classList.add("Coral");
      } else {
        const bit = (rowState[rowName] >> i) & 1;
        if (bit) btn.classList.add("on");
      }

      // Mark targeted box if matches currentTarget
      const label = getTargetLabel(rowName, i);
      if (currentTarget === label) {
        btn.classList.add("targeted");
      }

      btn.onclick = () => {
        if (targetMode) {
          // In target mode, clicking sets target and syncs to NT
          currentTarget = label;
          ntClient.addSample(toRobotPrefix + targetBoxTopicName, currentTarget);
          clearTargetHighlights();
          btn.classList.add("targeted");
          console.log("Target set to:", currentTarget);
          return;
        }

        // Normal mode: toggle bits
        if (rowName === "Hybrid") {
          let current = (rowState.Hybrid >> (i * 2)) & 0b11;
          current = (current + 1) % 3;
          rowState.Hybrid &= ~(0b11 << (i * 2));
          rowState.Hybrid |= (current << (i * 2));
          ntClient.addSample(toRobotPrefix + "HybridRow", rowState.Hybrid);
        } else {
          rowState[rowName] ^= (1 << i);
          ntClient.addSample(toRobotPrefix + rowName + "Row", rowState[rowName]);
        }
        renderButtons();
      };

      rowEl.appendChild(btn);
    }
  });
}

function getTargetLabel(rowName, index) {
  // Index is 0-based left to right, label is 1-based right to left visually
  // Flip index for label (rightmost button is 1)
  const visualIndex = index;

  if (rowName === "Hybrid") {
    // Hybrid row: label with Cone/Cube is for bitfield state
    const val = (rowState.Hybrid >> (index * 2)) & 0b11;
    let typeStr = "";
    if (val === 1) typeStr = "Algae";
    else if (val === 2) typeStr = "Coral";
    else typeStr = "None";
    return `Hybrid ${typeStr} ${visualIndex + 1}`;
  }
  return `${rowName} ${visualIndex + 1}`;
}