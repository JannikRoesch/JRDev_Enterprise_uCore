function saveConfig() {
  const json = document.getElementById("configJson").value;
  fetch("/save", {
    method: "POST",
    headers: {
      "Content-Type": "application/json"
    },
    body: json
  }).then(res => {
    if (res.ok) alert("Konfiguration gespeichert.");
    else alert("Fehler beim Speichern.");
  });
}

function startOTA() {
  const url = document.getElementById("otaUrl").value;
  fetch("/ota", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ url })
  }).then(res => {
    if (res.ok) alert("OTA gestartet.");
    else alert("OTA fehlgeschlagen.");
  });
}