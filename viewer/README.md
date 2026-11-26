# Visualizzatore UAV offline

Applicazione desktop Python per riprodurre posizione e assetto da CSV locali.
È indipendente dal simulatore: non compila, avvia o modifica il C++ e non integra
la dinamica. Il modello procedurale richiama un quadrotore Mavic 3, con quattro
bracci, dischi delle eliche, fusoliera e camera anteriore.

## Installazione e avvio

Dalla directory principale del progetto, con Python **3.11 o successivo**:

```bash
python3 -m venv viewer/.venv
viewer/.venv/bin/python -m pip install -r viewer/requirements.txt
viewer/.venv/bin/python viewer/view.py outputs/uav1_log.csv
```

Su Windows, usare `viewer\.venv\Scripts\python.exe` al posto di
`viewer/.venv/bin/python`. Sono necessari un desktop e driver grafici compatibili
con VTK/OpenGL. Installate le dipendenze, il viewer funziona senza connessione.
L'ambiente in `viewer/.venv` è separato dall'eventuale `.venv` del simulatore.

Senza argomento CSV si apre una finestra di selezione. **Ctrl+O** (Cmd+O su macOS)
consente di cambiare simulazione nella stessa finestra. La riproduzione parte in pausa.

```bash
# Solo controllo del file, senza librerie grafiche; richiede NumPy e SciPy.
viewer/.venv/bin/python viewer/view.py outputs/uav1_log.csv --validate

# Posa iniziale diversa da zero.
viewer/.venv/bin/python viewer/view.py outputs/uav1_log.csv --time 10

# Immagine di una posa (richiede il backend grafico, anche in offscreen).
viewer/.venv/bin/python viewer/view.py outputs/demo_yaw90.csv --time 4 --screenshot /tmp/uav-yaw90.png
```

## CSV e convenzioni

Il formato predefinito è quello ufficiale del simulatore (`outputs/uav1_log.csv`):
UTF-8, virgole come separatore, punto decimale, intestazione obbligatoria.
Il file si apre direttamente, senza conversioni o configurazioni aggiuntive:

```csv
Time,Position_ENU[0],Position_ENU[1],Position_ENU[2],Velocity_ENU[0],Velocity_ENU[1],Velocity_ENU[2],Quaternion_ENU_body[0],Quaternion_ENU_body[1],Quaternion_ENU_body[2],Quaternion_ENU_body[3],Omega_body[0],Omega_body[1],Omega_body[2]
0,0,0,0,0,0,0,1,0,0,0,0,0,0
0.005,0,0,0.01,0,0,2,1,0,0,0,0,0,0
```

- Posizione del **centro di massa** in metri nel riferimento ENU: X East, Y North, Z Up.
- Tempo in secondi, strettamente crescente; può iniziare da un valore diverso da zero
  e avere campionamento non uniforme. Un solo campione è visualizzato staticamente.
- Quaternioni **Hamilton**: `Quaternion_ENU_body[0..3]` contiene **w,x,y,z**,
  come scritto dal simulatore (scalare prima), con trasformazione:

  ```text
  vettore_ENU = R_ENU_from_body(q) @ vettore_body
  punto_ENU = posizione_CoM_ENU + R_ENU_from_body(q) @ punto_body
  ```

- Non viene applicata alcuna inversione implicita per la parola “passivo”.
  Se il file registra invece una mappa ENU→body, impostare
  `quaternion_direction = "enu_to_body"` nella configurazione.
- SciPy riceve i componenti riordinati come x,y,z,w; la matrice ottenuta è usata
  direttamente per il modello. Una convenzione JPL o una diversa definizione
  algebrica richiede un adattatore esplicito: non si deduce automaticamente dai dati.
- Il body è destrorso con **X avanti, Y sinistra, Z alto**. Z alto è coerente con
  l'header UAV attuale; X avanti e Y sinistra sono assunzioni grafiche dichiarate.
- Con q=(1,0,0,0) body ed ENU coincidono: la prua punta a East.
  Con q=(sqrt(0.5),0,0,sqrt(0.5)) la prua punta a North e Y body a West.
- Il CoM è l'origine grafica del modello e della terna body, indipendentemente
  dalla posizione del centro del suo ingombro. Non viene ricavato dalla mesh.

Colonne facoltative, sempre in gruppi completi di tre:

```text
Velocity_ENU[0], Velocity_ENU[1], Velocity_ENU[2]
Omega_body[0], Omega_body[1], Omega_body[2]
```

La velocità è in ENU; la velocità angolare è espressa nel body, in rad/s.
Le loro norme sono mostrate nella telemetria, senza frecce nella scena. Non
servono per ricostruire posizione o assetto. Le altre colonne sono ignorate.

I quaternion vengono normalizzati; deviazioni di norma oltre l'1% producono
un avviso. Sono rifiutati quaternion degeneri, NaN/infinito, campi numerici errati,
righe incomplete e tempi duplicati o decrescenti. Gli errori delle righe indicano
il numero di riga del CSV. I file sono aperti esclusivamente in lettura.

## Controlli e visualizzazione

- **▶** avvia o riprende; **⏸** mette in pausa. I pulsanti hanno solo il simbolo.
- Spazio alterna play e pausa; al termine del file, ▶ riparte dall'inizio.
- Velocità di riproduzione da 0.25× a 4×.
- Due caselle per attivare/disattivare **Terna body** e **Traiettoria**.
- Telemetria sempre visibile: tempo, posizione ENU, quaternion e norme delle
  velocità lineare/angolare, se presenti nel CSV.
- Mouse per orbitare, zoom e pan. La modalità camera iniziale è nel TOML.

La terna body usa X rosso, Y verde, Z blu: lunghezza predefinita 0,20 m,
aste sottili e punte compatte. `model.axes_length_m` ne controlla la lunghezza.
Gli assi inerziali sono fissi nell'origine ENU e mantengono una lunghezza
indipendente (`model.world_axes_length_m`, default 0,32 m).

La griglia resta **fissa a quota zero in tutte le viste**. Usa due scale di linee
allineate alle coordinate ENU, con dissolvenza continua tra celle fini e grandi
quando cambia lo zoom. I bordi sfumano e, quando il terreno è lontano rispetto
all'inquadratura, la griglia scompare gradualmente. Non ci sono piani che seguono
il drone, cambi di livello o scritte sovrapposte. Inquadrando nuovamente il terreno
o allargando la vista, la griglia torna visibile. La quota reale del CoM rimane nella
telemetria. La geometria della griglia viene creata una volta e ha costo limitato
anche per traiettorie lunghe o ad alta quota.

Il punto dorato identifica il CoM. La traiettoria completa è
attenuata e il segmento percorso è evidenziato. Non vengono disegnati collegamenti
attraverso lacune nei dati. Le scale spaziali sono uguali sui tre assi.

I dischi semitrasparenti rappresentano le eliche, senza inventare RPM non registrati.

Il rendering predefinito privilegia la fluidità: antialiasing disabilitato,
modello con meno poligoni e parti accorpate per materiale, trasformazione VTK
condivisa tra le parti del drone. La traccia usa al massimo 4000 punti grafici
(`display.max_trail_points`); tutti i campioni originali restano disponibili per
l'interpolazione della posa. La telemetria si aggiorna a 20 Hz durante il playback; pausa, velocità e
comandi dell'utente si aggiornano subito.

Il clock monotono governa il playback; il timer Qt richiede circa 60 aggiornamenti
al secondo. La fluidità effettiva dipende da GPU, backend e dimensioni dei dati.
La posizione è interpolata linearmente e l'assetto con SLERP, senza angoli di Eulero,
filtri o estrapolazione oltre il file. Al termine la riproduzione si ferma.
SLERP sceglie la rotazione più breve: il CSV deve campionare le rotazioni abbastanza
spesso da evitare ambiguità maggiori di 180° tra due campioni.

Gli intervalli oltre `gap_threshold_s` (default 1 s) sono considerati lacune.
Il playback si ferma al campione precedente e mostra un avviso. Un nuovo clic
su ▶ riprende dal primo campione dopo la lacuna, senza inventare movimento
nell'intervallo mancante. Per registrazioni legittimamente lente, aumentare
la soglia nella configurazione.

## Configurazione

`viewer/viewer.toml` contiene mappatura delle colonne, direzione del quaternion,
dimensioni del modello, terne e camera. La configurazione predefinita
è risolta rispetto allo script, anche se lo si avvia da un'altra directory.
Un override può contenere soltanto le opzioni da cambiare:

```toml
[columns]
time = "time"
position = ["east", "north", "up"]
quaternion = ["q0", "q1", "q2", "q3"] # sempre scalare prima

[data]
gap_threshold_s = 2.0

[model]
arm_m = 0.25
```

```bash
viewer/.venv/bin/python viewer/view.py outputs/flight.csv --config /percorso/custom.toml
```

Dimensioni e ordine dei motori iniziali riprendono il progetto: a=0.19/sqrt(2) m;
M1=(+a,+a,0), M2=(-a,+a,0), M3=(-a,-a,0), M4=(+a,-a,0).
Sono parametri grafici autonomi: il viewer non importa né analizza il C++.

## CSV di prova

Il generatore crea nove file `outputs/demo_*.csv` a 20 Hz con i nomi di colonna
del log ufficiale; le demo senza vettori omettono le colonne facoltative. Il rendering interpola
tra i campioni. Per rigenerarli, anche senza dipendenze Python aggiuntive:

```bash
python3 viewer/examples/generate_demo.py
```

Il generatore **non sovrascrive file esistenti**. I vecchi CSV demo con colonne
`t_s,x_enu_m,...,qw,qx,qy,qz` si aprono con la mappatura esplicita:

```bash
viewer/.venv/bin/python viewer/view.py outputs/demo_flight.csv --config viewer/examples/legacy_columns.toml
```

Per generare esempi nel formato attuale in una nuova directory:

```bash
python3 viewer/examples/generate_demo.py --output-dir /tmp/uav-demo
```

| File | Controllo |
| --- | --- |
| `demo_hover.csv` | Posizione e assetto costanti |
| `demo_translation.csv` | Traslazioni successive lungo East, North e Up |
| `demo_yaw90.csv` | Prua da East a North, CoM fermo |
| `demo_roll90.csv` | Rotazione positiva di 90° attorno a X body iniziale |
| `demo_pitch90.csv` | Rotazione positiva di 90° attorno a Y body iniziale |
| `demo_q_sign.csv` | Segni q/-q alternati senza scatti di assetto |
| `demo_circle.csv` | Circuito con prua tangente, quota variabile e dati di velocità |
| `demo_flight.csv` | Traiettoria, roll, pitch, yaw e velocità lineare/angolare |
| `demo_gap.csv` | Lacuna tra t=3 s e t=6 s, arresto prima dell'interpolazione |

I CSV in `outputs/` restano esclusi da Git dal `.gitignore` già esistente.
Il generatore versionabile permette di ricrearli.

## Test

```bash
viewer/.venv/bin/python -m pip install -r viewer/requirements-dev.txt
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 viewer/.venv/bin/python -m pytest viewer/tests -q

# Solo con dipendenze e backend grafico disponibili:
UAV_VIEWER_RENDER_TESTS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 viewer/.venv/bin/python -m pytest viewer/tests -q
```

L'opzione che disabilita il caricamento automatico dei plugin evita interferenze
con i plugin ROS presenti in alcuni ambienti. I test numerici verificano direzione
delle trasformazioni, campioni originali, SLERP, q/-q, normalizzazione e errori CSV.
I test del playback usano un clock controllato per verificare pause, velocità,
callback in ritardo, fine file e lacune. I test grafici aggiungono un rendering
offscreen e una prova dei controlli Qt.

Verifiche effettuate nell'ambiente di sviluppo: 72 test superati, inclusi test
numerici, rendering offscreen e controlli Qt (con QT_QPA_PLATFORM=offscreen).
Validato il log attuale `outputs/uav1_log.csv`: 4000 campioni da 0 a 19,995 s,
senza lacune; ispezionati rendering vicini al terreno, in quota e panoramici.
I test verificano anche continuità delle dissolvenze, geometria della griglia
costante fino a 100 km, trasformazioni del modello, terna body compatta,
assenza dei vettori, controlli essenziali, telemetria live, lacune e cambio CSV.
Il controllo interattivo su desktop resta da confermare.

La chiusura assegna esplicitamente i controlli al loro parent Qt, ferma e
scollega il timer, disabilita le callback di rendering e rilascia VTK prima
di distruggere la finestra. La distruzione avviene mentre QApplication è
ancora attiva, senza affidarsi alla pulizia finale di Python. I test includono
anche callback tardive, chiusure ripetute e processi separati che esercitano
la CLI e l'uscita dell'interprete, in pausa, durante il playback e tramite quit
dell'applicazione. Le prove automatiche sono offscreen; il crash del desktop
Wayland è stato analizzato tramite il core dump, ma non riprodotto qui.

Benchmark riproducibile, senza avviare il simulatore:

```bash
QT_QPA_PLATFORM=offscreen MPLCONFIGDIR=/tmp/uav-mpl viewer/.venv/bin/python viewer/examples/benchmark_render.py --csv outputs/uav1_log.csv
```

Il comando riporta la mediana di aggiornamento e rendering e il 95° percentile
del tempo totale per frame a 1000×700. Include volo vicino a terra, salita fino
a circa 1000 m, discesa, traiettoria estesa 10 km, panoramica, vista assetto e CSV.
Nell'ambiente di sviluppo il tempo mediano complessivo scende da circa 28–39 ms
a circa 8–11 ms per frame. Il benchmark esclude caricamento e avvio e misura
il rendering offscreen; la fluidità della finestra dipende anche da GPU,
risoluzione e carico della macchina. Non è una garanzia di 60 fps su ogni sistema.
VTK emette avvisi di deprecazione relativi a NumPy 2.5, senza fallimenti nei test.

## Struttura e riferimenti

`data.py`: CSV e pose; `playback.py`: clock e timeline; `model.py`: geometria locale;
`scene.py`: rendering e camere; `grid.py`: griglia adattiva; `app.py`: interfaccia; `config.py`: TOML.

- PyVistaQt: https://qt.pyvista.org/api_reference.html
- SciPy Rotation: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
- SciPy SLERP: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html
