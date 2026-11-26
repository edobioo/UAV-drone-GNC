# Piano del visualizzatore UAV offline

Stato: implementazione presente in questa directory. Il simulatore C++ rimane indipendente.
Per i comandi e lo stato effettivo delle verifiche vedere README.md. Rendering
offscreen e controlli Qt verificati; fluidità su desktop ancora da confermare.

## 1. Scelta tecnica

App desktop Python 3 con:
- NumPy per array e lettura numerica, csv della libreria standard per intestazioni e diagnostica delle righe;
- SciPy Rotation e Slerp per conversioni e interpolazione dell'assetto;
- PyVista/VTK per rendering 3D e geometria procedurale;
- PyVistaQt QtInteractor e PySide6 per finestra, timeline e controlli.

L'applicazione usa soltanto file locali dopo l'installazione delle dipendenze.
Un ambiente virtuale e requirements.txt separati evitano modifiche alla build C++.
Il rendering richiede un ambiente desktop e un backend grafico compatibile con VTK.

## 2. Contratto dei dati

CSV UTF-8 con intestazione, separatore virgola e punto decimale. Una riga per campione:

Il formato predefinito segue `outputs/uav1_log.csv`:

- `Time`: tempo della simulazione in secondi, anche non uniforme e con origine diversa da zero.
- `Position_ENU[0..2]`: posizione del CoM in ENU, in metri.
- `Quaternion_ENU_body[0..3]`: Hamilton, ordine w,x,y,z, trasformazione body→ENU.
- `Velocity_ENU[0..2]`: velocità in ENU, in m/s (gruppo facoltativo).
- `Omega_body[0..2]`: velocità angolare body, in rad/s (gruppo facoltativo).
- Ordine delle colonne libero: si identificano per nome.
- `viewer.toml`, letto con tomllib su Python >= 3.11, può mappare nomi diversi senza alterare i CSV.
- Stesso file per dimensioni grafiche, dimensioni delle terne e opzioni camera.
- Colonne aggiuntive ignorate se non utilizzate; nessuna integrazione dei vettori.

Controlli al caricamento: colonne presenti, valori finiti, tempi strettamente crescenti,
righe coerenti e quaternioni di norma non nulla. Piccole derive di norma sono corrette;
deviazioni importanti sono segnalate e quaternioni degeneri rifiutati con numero di riga.
Tempi duplicati o decrescenti sono rifiutati, senza riordinamento silenzioso.
Un solo campione è visualizzabile come posa statica. Grandi intervalli temporali sono
segnalati come possibili lacune: il playback si ferma al bordo finché non si abilita
esplicitamente l'interpolazione di quella lacuna.

## 3. Convenzioni geometriche e quaternion

Inerziale destrorso: X East, Y North, Z Up.
Nel codice esistente il body ha Z Up; per la geometria si assume X avanti, Y sinistra.
Quest'ultima assunzione va confermata contro le convenzioni della futura dinamica.

La definizione operativa, indipendente dalle etichette attivo/passivo, è:

    v_ENU = R_ENU_from_B(q) @ v_B
    p_ENU = position_CoM_ENU + R_ENU_from_B(q) @ p_B

Si usa la matrice che mappa body in ENU. Nessuna inversione automatica per la parola
"passivo". Con quaternion Hamilton, la conversione standard di SciPy viene adattata
all'ordine qw,qx,qy,qz dichiarato. Una convenzione algebrica diversa richiederà un
adattatore esplicito e un caso di controllo, non un cambio di segno per tentativi.
La mappatura opzionale di un file ENU->body usa un'inversione esplicita e documentata.

Verifica fondamentale: per q=(cos(pi/4),0,0,sin(pi/4)) nella convenzione Hamilton
proposta, X body deve puntare a North e Y body a West. Con q identità le terne coincidono.
Se il CSV rappresenta diversamente questo caso, si corregge l'adattatore d'ingresso.

## 4. Modello quadrotore

Geometria procedurale semplificata ispirata al Mavic 3: fusoliera grigia allungata,
prua riconoscibile con piccola camera/gimbal, quattro bracci e motori, dischi delle eliche.
Nessun asset esterno necessario. Il riferimento locale della geometria è nel CoM;
non si assume che il centro del bounding box coincida con il CoM.

La prima geometria usa un braccio di 0.19 m e le posizioni già presenti nel progetto:
M1=(+a,+a,0), M2=(-a,+a,0), M3=(-a,-a,0), M4=(+a,-a,0), a=0.19/sqrt(2).
Le dimensioni restano configurabili nel viewer senza importare o analizzare il C++.
Il modello è illustrativo, non una riproduzione dimensionale certificata del Mavic 3.

Terna body compatta (0,20 m, aste sottili e punte ridotte), centrata nel CoM: X rosso, Y verde, Z blu, con etichette.
Tutte le parti condividono una trasformazione rigida assoluta. I dischi delle eliche
sono una rappresentazione visiva: senza RPM registrati non si mostra una rotazione
che pretenda di rappresentare lo stato dei motori.

## 5. Scena e interazione

- Griglia orizzontale ENU a quota zero, assi inerziali fissi e scala metrica uniforme.
- Traiettoria completa tenue e traccia già percorsa evidenziata, opzionalmente nascondibili.
- Marcatore del CoM e terna body sempre coerenti con la posa del modello.
- Mouse per orbitare, zoom e pan; modalità camera iniziale configurata nel TOML.
- Interfaccia minima: due pulsanti a simbolo ▶/⏸, velocità 0.25x–4x,
  due interruttori per terna body e traiettoria, telemetria sempre visibile.
- Indicazione aggiornata di tempo simulato, durata, posizione ENU, quaternion
  e norme delle velocità opzionali. Nessuna freccia per le velocità.
- Nessuna intestazione CSV, barra di avanzamento, campo temporale, loop,
  pulsante campione o selettore camera. Ctrl+O apre un altro CSV.
- Lacune: pausa esplicita con avviso; un nuovo Play riprende dal campione
  successivo alla lacuna, senza interpolarla.

La prima versione non include grafici temporali aggiuntivi né esportazione video.
L'esportazione a frame rate fisso può essere aggiunta dopo aver validato il playback.

## 6. Playback fluido

Obiettivo: circa 60 frame/s su una macchina con backend grafico adeguato, indipendentemente
dalla frequenza dei campioni. Non è una garanzia per qualsiasi file o hardware.

- Tempo corrente derivato da clock monotono, istante di avvio e fattore di velocità.
- Qt richiede un aggiornamento circa ogni 16 ms; eventuali callback in ritardo non
  rallentano artificialmente il tempo simulato.
- Posizione interpolata linearmente per default, senza overshoot introdotto da spline.
- Assetto interpolato con SLERP, senza passare da angoli di Eulero.
- Gestione coerente di q e -q per non introdurre rotazioni spurie.
- Nessuna estrapolazione oltre l'intervallo del file; l'ultimo frame coincide con l'ultimo campione.
- Nessuna integrazione della dinamica e nessun filtro che alteri i campioni registrati.
- Mesh create una sola volta; le parti del drone condividono una matrice VTK.
- Griglia fissa a quota zero con due scale in dissolvenza continua e scomparsa
  graduale quando il terreno è lontano; nessun aggiornamento a livelli di quota.
- Rendering leggero senza supersampling, modello semplificato e traccia con costo
  limitato dal numero massimo di punti grafici, indipendente dai campioni del CSV.
- Telemetria a 20 Hz durante il playback, rendering governato dal clock monotono.
- Controlli con parent Qt esplicito; shutdown idempotente: timer e callback
  disattivati, VTK rilasciato e finestra distrutta prima dell'uscita di Qt.
- Verificare anche l'uscita dell'interprete in processi separati, oltre alla
  chiusura della finestra all'interno dei test Qt.
- Eventuale riduzione dei punti della traccia esclusivamente grafica; dati di posa conservati.

SLERP segue il percorso angolare più breve tra campioni: il CSV deve campionare
l'assetto abbastanza spesso da non rendere ambigue rotazioni maggiori di 180 gradi
tra due righe. Un frame rate alto non recupera manovre assenti dai dati.

## 7. Struttura prevista

```text
viewer/
  AGENTS.md
  PLAN.md
  README.md
  requirements.txt
  viewer.toml
  view.py                 # entry point CLI
  uav_viewer/
    data.py               # lettura e validazione
    playback.py           # tempo e interpolazione
    model.py              # geometria nel body
    scene.py              # attori, trasformazioni e camere
    app.py                # finestra Qt e controlli
  examples/
    generate_demo.py      # CSV sintetici riproducibili, senza simulatore
  tests/
    test_data.py
    test_pose.py
    test_playback.py
```

Comando previsto dalla root: python viewer/view.py outputs/flight.csv
Opzione prevista: --config viewer/viewer.toml
La configurazione predefinita si risolve rispetto allo script, non alla directory corrente.

## 8. Ordine di realizzazione e criteri di accettazione

1. Contratto CSV e nucleo matematico: loader e campionamento di posa indipendenti dalla GUI.
   Test per errori dei dati, identità, rotazioni di 90 gradi sui tre assi, q/-q,
   traslazione pura, tempi non uniformi e riproduzione esatta dei campioni originali.
2. Scena statica: modello, CoM, terne, griglia e traiettoria. Verifica visiva che
   rotazioni pure non traslino il CoM e che nessun asse sia invertito o specchiato.
3. Animazione e controlli: Play, pausa, velocità, terna/traiettoria e telemetria live.
   Verifica di pausa, ripresa, fine file, lacune e velocità basata sul tempo
   anche con callback irregolari.
4. Rifinitura: antialiasing, materiali semplici, traccia efficiente, messaggi leggibili,
   telemetria e controlli essenziali. Prova di fluidità su un CSV sintetico lungo.
5. Consegna: README con installazione, formato e convenzioni, generatore demo e
   istruzioni per una verifica visiva riproducibile. Prova desktop reale; gli eventuali
   limiti di verifica del backend grafico vanno riportati esplicitamente.

Esempi necessari: hovering, traslazione lungo E/N/U, yaw di 90 gradi, roll/pitch isolati,
traiettoria circolare e sequenza con quaternion alternati q/-q.

## Riferimenti tecnici

- https://qt.pyvista.org/api_reference.html
- https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
- https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html
