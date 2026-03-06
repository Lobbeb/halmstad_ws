# Praktiska Krav + To-Do (Halvtid 09/03)

Den har filen foljer Google-dokumentets ordning och text sa nara som mojligt.

- Huvudpunkterna ar era riktiga to-do for halvtid.
- Under varje huvudpunkt ligger korta tekniska delsteg (substeps) sa det gar att exekvera i repo.
- Sadant som ligger utanfor halvtidskarnan hamnar under Extra (om tid finns).

## ROS2-GZ (huvudlista i ordning)

### 1) Vanta in Tommys uppdatering i GitLab med ny UAV-modell och en controller-vag som gor att UAV:n kan styras pa ett mer realistiskt satt.

Delsteg:
- Integrera upstream-kod utan att bryta befintlig follow-stack.
- Verifiera att bade setpose-backend och controller-backend kan startas fran samma run-harness.
- Sakerstall att kameratopics och controller-topics finns enligt kontrakt.

### 2) Bestam tydligt styrkedjan: vi skickar inte manuella kommandon till UAV:n, utan vi genererar en plan/trajectory som skickas till en controller som sedan styr UAV:n.

Delsteg:
- Las kontrollkedjan: `observation -> estimate -> follow-intent -> backend-adapter -> actuation`.
- Hall samma planner-logik oavsett backend; backend far bara oversatta intent.
- Undvik joystick/manuell styrning i experimentkorningar.

### 3) Las grundkravet: UGV ska inte dela sin state till UAV:n. Foljning maste bygga pa vad UAV:n sjalv kan observera.

Delsteg:
- Kor policy-lage dar UAV-foljning inte far bero pa UGV-odom (`uav_only` + pose/estimate-kedja).
- Fail-fast om fel mode kombineras (t.ex. odom i strikt observationslage).
- Behall odom-lage endast som debug/baseline, inte som krav for UAV-follow.

### 4) Anvand YOLO for att fa en relativ uppskattning av UGV:s position (och om mojligt hastighet) utifran kameradata.

Detta innebar att vi stabiliserar hela observationskedjan (`kamera -> YOLO -> leader_estimate/leader_estimate_status`) sa att UAV:n kan folja robust utan beroende av UGV-odom i `uav_only`-lage. Vi verifierar aven kontinuerlig estimate- och kommandostrom i bada backends (`setpose` och `controller`) for jamforbara och reproducerbara korningar.

Delsteg:
- Input: `/dji0/camera0/image_raw` + `/dji0/camera0/camera_info` (ev. depth senare).
- Output: `/coord/leader_estimate` + `/coord/leader_estimate_status`.
- Lagg robusthet for tappad detektion: quality-gate, hold, reacquire.
- Sakerstall att estimate driver follow i bada backends med samma loggkontrakt.

### 5) Stabilisering/prediktion: lagg ett Kalmanfilter ovanpa YOLO-matningarna for att klara brus och tappade detektioner och for att kunna prediktera kort framat.

Delsteg:
- Filtrera estimate till stabil `pos/vel`.
- Anvand kort prediktion vid tillfalliga dropouts.
- Logga filterstatus sa ra vs filtrerad effekt kan visas i resultat.

### 6) Path-generering: anvand den estimerade staten (pos/vel) for att generera en folj-punkt och en rimlig rorelseplan som controllern kan folja.

Delsteg:
- Berakna foljpunkt/standoff fran estimerad state.
- Begransa med hastighet/acceleration/deadband for stabil rorelse.
- Publicera planerad intent till vald backend, inte manuella direkta ryck-kommandon.

### 7) Hybrid som novelty: kombinera en robust baseline (follow+leash) med perception+Kalman, dar systemet kan falla tillbaka nar perceptionen ar svag.

(Kanske ocksa lagga in Edisons ide om PHERMONE fran hans papper som extra novelty om tid finns.)

Delsteg:
- Definiera fallback-villkor (svag perception, stale, lag confidence).
- Vaxla med hysteresis sa systemet inte flappar.
- Logga mode/state-vaxlingar for jamforelse i resultat.

### 8) Kommunikationssparet (Ruben): utred hur RSSI/natkvalitet kan anvandas som extra signal for leashing eller vaxling i hybridlogiken, och hur det kan kopplas in via OMNeT senare.

Delsteg:
- Definiera comm-signalgranssnitt redan nu (syntetisk forst).
- Hall integrationen los kopplad sa OMNeT kan kopplas in senare utan omskrivning.
- Dokumentera vilka trosklar/indikatorer som ska styra vaxling.

### 9) Fardigstall communication-based leashing som extension: behall dagens distansbaserade leash som baseline, men koppla in en faktisk lankkvalitetssignal (t.ex. RSSI/PRR/latens) i leashing-logiken med tydliga trosklar och hysteresis, och logga lankmatt + leashing-state for jamforelse.

(Starta med en syntetisk lanksignal i ROS och byt senare till OMNeT++/natlager nar det ar pa plats.)

Delsteg:
- Implementera comm-aware leash som pabyggnad, inte ersattning av baseline.
- Sakerstall att baseline-lage kan koras oforandrat for A/B-jamforelse.
- Logga comm-signal + leash-state i samma bag/meta-kontrakt.

### 10) Kamera-/vinkeljusteringar for att undvika tappad vision

("Men fick andra kameran och vinkeln ... annars predictade den ingenting ... flog forbi UGV ... tappade vision")

Delsteg:
- Justera kamera pitch/yaw/FOV sa UGV halls i synfalt vid normal foljning.
- Lagg aktiv camera-lock dar relevant sa mal halls centrerat.
- Verifiera i GUI + status att `NO_DET` minskar och reacquire fungerar.

### 11) Patcha OMNeT-bridge for flera UAVs i ROS2

Delsteg:
- Sakerstall topic-namning/namespace per UAV.
- Verifiera att flera UAV-instanser inte krockar i bridge.
- Dokumentera vad som ar fixat vs kvarvarande begransningar.

## OMNeT++

### 12) Patcha OMNeT sa flera UAVs kan spawna

Delsteg:
- Multi-UAV spawn utan manuella specialsteg.
- Stabil init/teardown mellan runs.

### 13) Satt upp natverkstopologin for multi-UAV ad-hoc

Delsteg:
- Definiera forsta testtopologi (enkel men reproducerbar).
- Exponera relevanta natmatt till ROS-sidan for jamforelse.

## Extra (om tid finns)

### E1) Las en liten run-matris for halvtid

Mal:
- baseline (odom + distans-leash)
- hybrid (YOLO + KF med fallback)
- comm-leash (lankkvalitet in i leash)

Delsteg:
- Fa korningar men tack alla pastaenden ni vill gora.
- Samma loggkontrakt i alla lagen.

### E2) Frys metrics + loggning en gang

Minst:
- tracking error / leash-brott
- estimator-uptime (dropouts)
- enkel smoothness pa kommandon

Delsteg:
- Samma rosbag-topics och samma `meta.yaml`-falt i alla lagen.
- Skriv run-ID i resultat sa allt ar sparbart.

### E3) Lagg in en repeterbar storning

Alternativ:
- forced YOLO-dropouts/occlusion
- kontrollerad link-quality dip (syntetisk forst)

Delsteg:
- Visa att hybrid/comm-leash ger robusthet, inte bara det funkar.
- Kor minst en A/B-jamforelse med samma setup.

## Snabb status (uppdateras lopande)

- Klart: 1, 2, 3 (implementerat i repo)
- Pagaende: 4 (slutvalidering + viktjamforelse)
- Nast pa tur: 5, 6
- Efter det: 7, 8, 9
- Sedan: 11, 12, 13 (OMNeT-sparet)
