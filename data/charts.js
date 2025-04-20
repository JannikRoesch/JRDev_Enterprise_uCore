let chart;

window.addEventListener("load", () => {
  const ctx = document.getElementById("liveChart").getContext("2d");
  chart = new Chart(ctx, {
    type: "line",
    data: {
      labels: [],
      datasets: [{
        label: "Sensorwert",
        data: [],
        fill: false,
        borderColor: "#007bff",
        tension: 0.1
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { display: false },
        y: { beginAtZero: true }
      }
    }
  });
});

function updateChart(value) {
  const now = new Date().toLocaleTimeString();
  if (chart.data.labels.length > 50) {
    chart.data.labels.shift();
    chart.data.datasets[0].data.shift();
  }
  chart.data.labels.push(now);
  chart.data.datasets[0].data.push(value);
  chart.update();
}