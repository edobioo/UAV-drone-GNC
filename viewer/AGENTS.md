# Visualizzatore offline UAV

## Ambito
- Questa directory contiene esclusivamente strumenti Python di post processing.
- Non modificare src/, include/, Makefile o la dinamica C++ per implementare il viewer.
- I CSV di simulazione sono input in sola lettura. Non sovrascriverli.
- Consultare PLAN.md prima dell'implementazione. Il piano non implica che le funzionalità siano già realizzate.

## Convenzioni
- Inerziale ENU: X East, Y North, Z Up; distanze in metri, tempi in secondi.
- La posizione nei CSV è quella del centro di massa nel riferimento ENU.
- La trasformazione vincolante è v_ENU = R_ENU_from_B @ v_B.
- La posa della geometria è p_ENU = position_CoM_ENU + R_ENU_from_B @ p_B.
- Default proposto: quaternioni Hamilton, colonne qw,qx,qy,qz; documentare esplicitamente ogni conversione per SciPy.
- Non coniugare quaternioni soltanto perché descritti come passivi.
- Body con Z verso l'alto, coerente con include/uav.hpp. Per la geometria assumere X avanti e Y sinistra, rendendo esplicita questa assunzione nel README.
- Origine di modello e terna body nel CoM. L'assetto non deve modificare la posizione del CoM.

## Implementazione e verifica
- Dipendenze e ambiente virtuale separati dal simulatore; nessun servizio o download necessario durante la riproduzione.
- Separare caricamento CSV, interpolazione, modello geometrico e interfaccia.
- Creare le mesh una volta e aggiornare trasformazioni assolute; evitare rotazioni incrementali e ricostruzione per frame.
- Il tempo di playback deriva da un clock monotono, non dal numero di callback Qt.
- Non integrare velocità o accelerazioni per ricostruire pose già presenti nel CSV.
- Verificare convenzioni con identità, rotazioni note di 90 gradi, q/-q e traslazione pura prima di valutare l'estetica.
- Non interpretare lacune o errori nei dati come traiettorie valide senza una segnalazione esplicita.
