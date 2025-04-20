const socket = new WebSocket("ws://" + location.hostname + "/ws");

/* 
socket.onmessage = function (event) {
  appendLog(event.data);
};
*/

socket.onmessage = function (event) {
  appendLog(event.data);
  try {
    const parsed = JSON.parse(event.data);
    if (parsed.temp !== undefined) {
      updateChart(parsed.temp);
    }
  } catch (e) {
    // Nicht JSON
  }
};