# Firmware

## Initialisierung & Registrierung

Der Roboter verbindet sich beim Start mit dem Backend, um seine Verfügbarkeit bekannt zu geben. 

*   **Registrierung:** `POST http://<BACKEND_HOST>:<BACKEND_PORT>/table/register` mit Payload `{"port": <ROBOT_PORT>}`.
*   **Websocket:** Verbindung zu `ws://<BACKEND_HOST>:<BACKEND_PORT>/ws/robot/control`.

## Routen: Backend Roboter

| B2R Route     | Methode | Zweck für das Backend                                                                                                                                               | C2B Entsprechung          |
| :------------ | :------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------ | :------------------------ |
| **`/status`** | `GET`   | **Statusabruf:** Der Backend Server fragt diese Route kontinuierlich ab, um das **Robot Status Model** für alle verbundenen Clients zu aktualisieren (Telemetrie).  | `GET /status`             |
| **`/nodes`**  | `GET`   | **Ortsdaten:** Das Backend fragt die Liste der definierten Ankerpunkte ab (z.B. "Mensa", "Raum \#1", "Apotheke"), die in der UI zur Routenauswahl angezeigt werden. | `GET /nodes`              |
| **`/select`** | `POST`  | **Routenzuweisung:** Das Backend weist dem Roboter die **komplette Fahrtenlogik** zu.                                                                               | `POST /routes/select`     |
| **`/mode`**   | `POST`  | **Moduswechsel:** Schaltet den Roboter in den Modus (MANUAL, AUTO, IDLE) um. Wird für Lock-Management und Timeout genutzt.                                          | `POST/DELETE /drive/lock` |

### 📝 Payload Details

#### Status

- `GET /status`

```json
{
  "systemHealth": "OK",
  "batteryLevel": 85,
  "driveMode": "MANUAL",
  "cargoStatus": "IN_TRANSIT",
  "lastRoute": {
    "startNode": "Mensa",
    "endNode": "Raum #3"
  },
  "position": "Raum #3" //... TODO
}
```

#### Routenzuweisung

```json
{
  "startNode": "Mensa",
  "endNode": "Zimmer 101"
}
```

#### Modussteuerung

```json
// Payload für POST /mode
{
  "mode": "MANUAL",
}

// Lock freigeben (IDLE)
{
  "mode": "IDLE"
}
```

## Websocket

Für die Steuerung mit niedriger Latenz muss das Backend die Befehle des Clients sofort an den Roboter weiterleiten (Proxy).

| B2R Route                | Protokoll                                  | Zweck für das Backend                                                                   |
| :----------------------- | :----------------------------------------- | :-------------------------------------------------------------------------------------- |
| **`/ws/control/manual`** | WebSocket (oder dedizierter TCP/UDP-Kanal) | **Echtzeit-Steuerung:** Dient als Proxy für die direkten Steuerbefehledes Lock-Halters. |

### Sicherheit: Timeout-Logik

Die Logik des **Timeouts** sollte im Backend angesiedelt sein, aber dessen Auswirkung auf den Roboter erfolgt über die **`POST /control/mode`** Route:

1.  **Backend-Überwachung:** Das Backend startet den Timer, sobald es den Lock zuweist.
2.  **Timeout-Ereignis:** Läuft der Timer ab, sendet das Backend sofort den Befehl: `POST /control/mode` mit `{"mode": "IDLE"}` an den Roboter.
3.  **Client-Kommunikation:** Das Backend schließt dann die WS-Verbindung des Lock-Halters und sendet die `LOCK_RELEASED`-Nachricht an alle anderen verbundenen Clients.
