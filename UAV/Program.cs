using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Runtime.ConstrainedExecution;
using System.Threading;


namespace SIHA
{
    public class Coordinate
    {
        public double Latitude;
        public double Longitude;

        //Yeni bir konum nesnesi oluştururken enlem ve boylamı atanır.
        public Coordinate(double lat, double log)
        {
            Latitude = lat;
            Longitude = log;
        }
        // görüş alanı (açı hesaplama)
        public double AngleTo(Coordinate other)
        {
            double dLon = other.Longitude - this.Longitude;
            double dLat = other.Latitude - this.Latitude;

            double angle = Math.Atan2(dLon, dLat) * (180 / Math.PI);
            return angle < 0 ? angle + 360 : angle;
        }
        // uçağın hareket etmesini sağlayan matematiksel ifadelerle tanımlı olan döngü
        public double DistanceTo(Coordinate target)
        {
            double R = 6371000;
            double lat1 = Latitude * Math.PI / 180;
            double lat2 = target.Latitude * Math.PI / 180;

            double deltaLat = (target.Latitude - Latitude) * Math.PI / 180;
            double deltaLon = (target.Longitude - Longitude) * Math.PI / 180;

            double a = Math.Sin(deltaLat / 2) * Math.Sin(deltaLat / 2) +
                Math.Cos(lat1) * Math.Cos(lat2) *
                Math.Sin(deltaLon / 2) * Math.Sin(deltaLon / 2);

            double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            return R * c;
        }
        // Uçağın hedefe varması için gerekli matematiksel ifadelerle tanımlı olan döngü
        public double BrearingTo(Coordinate target)
        {
            double lat1 = Latitude * Math.PI / 180;
            double lat2 = target.Latitude * Math.PI / 180;
            double deltaLon = (target.Longitude - Longitude) * Math.PI / 180;

            double y = Math.Sin(deltaLon) * Math.Cos(lat2);

            double x = Math.Cos(lat1) * Math.Sin(lat2) -
                Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(deltaLon);

            double brearing = Math.Atan2(y, x);
            return (brearing * 180 / Math.PI + 360) % 360;
        }
    }
    public class UAV
    {
        public Coordinate Position { get; set; }
        public Coordinate Target { get;  set; }
        private PIDController pidlat = new PIDController(0.1, 0.5, 0.2);
        private PIDController pidlon = new PIDController(0.1, 0.5, 0.2);
        private double Fuel = 450;
        private const double FuelConsumptionStep = 1.0;
        private int StepCount = 0;
        private Wind wind = new Wind();
        private List<UAVPoint> FlightPath = new List<UAVPoint>();
        private List<NoFlyZone> noflyzone = new List<NoFlyZone>();
        private bool IsManuelControl = false;
        private int manuelControlCounter = 0;
        private const int manuelControlMaxSteps = 10; // 10 adım sonra otomatik moda döner
        public List<EnemyZone> enemyZones = new List<EnemyZone>();
        private List<FriendZone> friendZones = new List<FriendZone>();
        private List<RadarZone> radarzone = new List<RadarZone>();
        private List<IRTarget> IRTargets = new List<IRTarget>();
        private List<EnemyJammer> jammers = new List<EnemyJammer>();
        public List<TargetObject> AttackTarget { get; set; } = new List<TargetObject>();
        public List<Obstacle> statickObstacle = new List<Obstacle>();
        public List<EMPZone> empZones = new List<EMPZone>();
        public List<Coordinate> learnedThereats = new List<Coordinate>();
        public List<Coordinate> MissionWayPoint { get; set; } = new List<Coordinate>();
        private double altitude = 100; //mevcut irtifa
        private const double SafeDistanceFromGround = 100; // En az 100m güvenli mesafe
        private LidarSensor lidar;
        public List<TargetObject> AttackTargett { get; set; } = new List<TargetObject>();
        public WeaponSystem WeaponSystem { get; set; } // EKLENDİ
        public Coordinate GetPosition()
        {
            return Position;
        }
        public double VisibilityRange { get; set; } = 500;  //metre
        public double ViewAngle { get; set; } = 45; //derece
        public double Heading { get; set; } = 90.0;
        public Coordinate ReturningBaseCoordinate { get; set; } = new Coordinate(39.920770, 32.854110);
        public bool IsReturningToBase { get; set; } = false;
        public bool IsFuselageDamage { get; private set; } = false;
        public bool IsEngineDamage { get; private set; } = false;
        public bool IsSensorDamage { get; private set; } = false;
        public bool isSensorOffline { get; private set; } = false;
        public bool HasRadarAbsorgingCoating { get; set; } = false;
        public double StealtFactor => HasRadarAbsorgingCoating ? 0.3 : 0.1;  //%70  daha az görünür
        private Random rdr = new Random();
        public double DamageLevel { get; set; } = 0.0;
        public bool IsAltitudeManualControl { get; set; } = false;
        private HashSet<string> KwownThreats = new HashSet<string>();
        private NightVision nightVision;
        bool wasInEnemyZone = false;
        bool wasInFriendZone = false;
        bool IsConnectionLost = false;
        bool empActive = false;
        bool empWarningGiven = false;
        int ConnectionLostCounter = 0;
        int ConnectionTimeOutLimit = 10; //10 adım sonra görev iptal edilsin
        int engineFailureCounter = 0;
        int sensorFailureCounter = 0;
        int bodyFailureCounter = 0;
        int empDuration = 5;
        int empCounter = 0;
        int radarExposureCounter = 0;
        int EMPDuration = 0;
        private bool isEvading = false;
        private int evadeStepCounter = 0;
        private const int MaxEvadeSteps = 4;
        private const double EvasiveTurnAngle = 90;
        private Random random = new Random();
        bool nightVisionMode { get; set; } = false;
        private int JammingCounter = 0;
        private const int MaxjammingAttemps = 3;
        public bool IsEMPActive { get; set; } = true;
        public List<NoFlyZone> noFlyZones { get; set; } = new List<NoFlyZone>();

        private int empDuractionCounter = 0;
        private const int EMP_DURATION_LIMIT = 5;  //5 adım boyunca sistemler kapalı

        // Uçağın gece görüşü
        public void InitializeNightVision()
        {
            Random rand = new Random();
            bool isNight = rand.Next(0, 2) == 2; // %50 ihtimalle gece
            double temp = rand.Next(-5, 35);   // -5°C ile 35°C arası sıcaklık

            nightVision = new NightVision(isNight, temp);
            nightVision.PrintStatus();
        }

        //Xunit Test
        // Basitleştirilmiş hedefe ulaşma kontrolü
        public bool HasReachedTarget()
        {
            double DistanceToTarget = Position.DistanceTo(Target);
            return DistanceToTarget <= 30;
        }
        // Test için hedefe manuel ulaşma fonksiyonu Xunit Test
        public void SimulateMoveToTarget()
        {
            Position = new Coordinate(Target.Latitude, Target.Longitude);  // direkt hedefe ulaş
        }
        // Uçak roketten kaçabilir yada Radar emiciler devreye girer ve radarı enzorbe eder (ama düşük bir ihtimalde olsa uçak vurulup düşebilir)
        private void CheckRadarDetectionInSAM()
        {
            foreach (var obj in AttackTarget)
            {
                if (obj.Type.ToLower().Contains("radar") && !obj.IsDestroyed && StepCount > 80)
                {
                    double distance = Position.DistanceTo(obj.Location);
                    if (distance < 1000)
                    {
                        bool evaded = random.Next(0, 1000) >= 30; //%70 başarı

                        if (evaded)
                        {
                            Console.ForegroundColor = ConsoleColor.Red;
                            // radar istasyonundan füze ateşlendi
                            Console.WriteLine("[SAM LAUNCHED] Missile fired from Radar Station.");
                            Console.ForegroundColor = ConsoleColor.Green;
                            // füzeden kurtuldun (en yüksek ihtimal)
                            Console.WriteLine("[MISSILE AVOIDED] UAV successfully evaded the missile.");
                            Console.ResetColor();
                            isEvading = true;
                            evadeStepCounter = MaxEvadeSteps;
                        }
                        else
                        {
                            Console.ForegroundColor = ConsoleColor.Red;
                            // uçak vuruldu
                            Console.WriteLine("[UAV DAMAGED] UAV was hit by SAM! Mission failed.");
                            Console.ResetColor();
                            Environment.Exit(0);
                        }
                        return;
                    }
                }
            }
        }
        // kaçınma manevrası
        private void PerformEvasiveManeuver()
        {
            Heading = NormalizeAngle(Heading + EvasiveTurnAngle);
            Console.ForegroundColor = ConsoleColor.Cyan;
            // ani manevra
            Console.WriteLine("[EVASIVE ACTION] Sudden maneuver executed to avoid SAM lock.");
            Console.ResetColor();
        }
        // Normal açı (uçağın görüş alanı)
        private double NormalizeAngle(double angle)
        {
            angle = angle % 360;
            return (angle < 0) ? angle + 360 : angle;
        }
        // Görüş alanı
        public bool IsTargetVisible(Coordinate target)
        {
            double distance = Position.DistanceTo(target);
            if (distance > VisibilityRange) return false;

            double targetAngle = Position.AngleTo(target);
            double heading = this.Heading;

            double angleDiff = Math.Abs(NormalizeAngle(targetAngle - heading));
            return angleDiff <= ViewAngle / 2;
        }
        // Radar bölgesinin kordinatları
        public void InitializeRadarZone()
        {
            radarzone.Clear();
            radarzone.Add(new RadarZone(39.9240, 32.8570, 0.3)); // 300 metre yarıçaplı radar alanı
            radarzone.Add(new RadarZone(39.9070, 32.8640, 0.25)); // başka bir radar alanı
        }
        // IR (ısı imzası) Uçağın algılanma ihtimali ?
        public void InitializeIrTargets()
        {
            IRTargets.Clear();
            IRTargets.Add(new IRTarget(39.9255, 32.8600, 0.6));  // güçlü IR imzası
            IRTargets.Add(new IRTarget(39.9265, 32.8620, 0.5));  // algılanmaz (çok düşük)
        }
        // EMP (sisteme (uçağa) elektiriksel yolla zarar vermeyi hedefleyen bi saldırı
        public void InitializeEMPZones()
        {
            empZones.Clear();
            empZones.Add(new EMPZone(39.9250, 32.8600, 100)); // 100 metre yarıçaplı EMP alanı
            empZones.Add(new EMPZone(39.9275, 32.8665, 80));  // başka bir bölge
        }
        // düşman bölgesinin kordinatları
        public void InitializeEnemyZones()
        {
            enemyZones.Add(new EnemyZone
            {
                Center = new Coordinate(39.9240, 32.8600),
                Radius = 300.0
            });
        }
        //3 den fazla tehdit oluşursa dön
        public bool IsThreatClustered(Coordinate currentCoordinate, double radius, int threshold)
        {
            int count = 0;

            foreach (var threat in learnedThereats)
            {
                if (currentCoordinate.DistanceTo(threat) <= radius)
                    count++;
            }
            foreach (var target in AttackTarget.Where(t => t.IsDestroyed))
            {
                if (currentCoordinate.DistanceTo(target.Location) <= radius)
                    count++;
            }
            return count >= threshold;
        }
#if DEBUG
        public Coordinate GetSafePoint(Coordinate current, Coordinate target, List<EnemyZone> enemyZones, List<NoFlyZone> noFlyZones)
#else
           
        private Coordinate GetSafePoint(Coordinate current, Coordinate target, List<EnemyZone> enemyZones, List<NoFlyZone> noFlyZones)
#endif
        {
            Random rand = new Random();
            for (int i = 0; i < 400; i++)
            {
                // Rastgele bir sapma yönü ve mesafesi (0.0005 ~ 50m gibi düşün)
                double offsetLat = (rand.NextDouble() - 0.5) * 0.001;
                double offsetLon = (rand.NextDouble() - 0.5) * 0.001;

                Coordinate candidate = new Coordinate(
                    current.Latitude + offsetLat,
                    current.Longitude + offsetLon
                    );

                bool IsSafe = true;
                // EnemyZone kontrolü
                foreach (var zone in enemyZones)
                {
                    if (zone.Center.DistanceTo(candidate) < zone.Radius + 50)
                    {
                        IsSafe = false;
                        break;
                    }
                }
                // NoFlyZone kontrolü
                foreach (var zone in noflyzone)
                {
                    if (zone.IsInside(candidate.Latitude, candidate.Longitude))
                    {
                        IsSafe = false;
                        break;
                    }
                }

                if (IsSafe)
                {
                    return candidate;
                }
            }
            return current;
        }


        public UAV(Coordinate position, Coordinate target)
        {
            this.Position = position;
            this.Target = target;
            this.altitude = 500; //başlangıç irtifası
            this.lidar = new LidarSensor();
        }
        // Dost bölgesinin kordinatları
        public void InitializeFriendZone()
        {
            friendZones.Add(new FriendZone
            {
                Center = new Coordinate(35.9215, 32.8555),
                Radius = 200.0
            });
        }
        //Uçağın karşılacağı engeller
        public void InitalizeStaticObstacle()
        {
            statickObstacle.Clear();
            statickObstacle.Add(new Obstacle(39.9240, 32.8570, 500, 50)); // Örnek: tepe
            statickObstacle.Add(new Obstacle(39.9250, 32.8650, 470, 40)); // Örnek: bina
        }
        public void CheckObstacleAvoidance()
        {
            foreach (var obs in statickObstacle)
            {
                if (obs.IsNear(Position))
                {
                    if (altitude < obs.Height + 30)  //çarpışma riski  varsa
                    {
                        Console.ForegroundColor = ConsoleColor.DarkYellow;
                        Console.WriteLine("[TERRAIN ALERT] Obstacle ahead! Gaining altitude to avoid collision...");
                        Console.ResetColor();

                        altitude += 20;  //acil yükseklik kazancı
                        return;
                    }
                }
            }
        }
        // Uçağın sistemlerinin bozulması durumu
        private void CheckForDamage()
        {
            int chance = rdr.Next(1000); // Hasarın oluşma ihtimali

            //Gövde hasarı 
            if (chance < 1 && !IsFuselageDamage)
            {
                IsFuselageDamage = true;
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[DAMAGE] Fuselage damaged! Stability compromised.");
                Console.ResetColor();
            }
            // Motor hasarı
            else if (chance < 2 && !IsEngineDamage)
            {
                IsEngineDamage = true;
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[DAMAGE] Engine failure! Speed reduced.");
                Console.ResetColor();
            }
            // sensor hasarı 
            else if (chance < 3 && !IsSensorDamage)
            {
                IsSensorDamage = true;
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[DAMAGE] Sensor malfunction! Accuracy reduced.");
                Console.ResetColor();
            }
        }
        private double GetSimulatedTerrainHeight(Coordinate pos)
        {
            // Basit bir dalgalı yüzey simülasyonu
            return 50 + 30 * Math.Sin(pos.Latitude * 10) * Math.Cos(pos.Longitude * 10);
        }
        // Sorun yok
        // Sinyalin geçiçi bir şekilde kesilmesi
        public bool AntiJammingCheck()
        {
            int chance = rdr.Next(100);  //random jam aralığı

            if (chance < 5) //%5 jam riski
            {
                JammingCounter++;

                Console.ForegroundColor = ConsoleColor.DarkRed;
                Console.WriteLine($"[JAMMING DETECTED] Signal interference detected! Attempting to switch channel... ({JammingCounter}/{MaxjammingAttemps})");
                Console.ResetColor();

                Thread.Sleep(1000); // Sinyal araması

                if (JammingCounter >= MaxjammingAttemps)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("[FAILURE] Unable to recover from jamming. Mission aborted.");
                    Console.ResetColor();
                    SetReturningToBase();  //iletişim yoksa üsse dön
                    return false;
                }
            }
            else
            {
                JammingCounter = 0; //Jamming yoksa sayaç sıfırlanır
            }
            return true;
        }
        public void AddJustAltitudeBasedOnTerrain()
        {
            double terrainHeight = GetSimulatedTerrainHeight(Position);  // Simülasyon
            double desiredAltitude = terrainHeight + 380.0;  // Yüzeyin 380 m üstü

            double minSafeAltitude = terrainHeight + 50.0;

            if (altitude < desiredAltitude - 5)
            {
                altitude += 5;  //yavaşça yüksel
                Console.WriteLine($"[Auto Altitude] Climbing to {altitude:F1} m (terrain: {terrainHeight:F1} m)");
            }
            else if (altitude > desiredAltitude + 5)
            {
                altitude -= 5;  //yavaşça alçal
                Console.WriteLine($"[Auto Altitude] Descending to {altitude:F1} m (terrain: {terrainHeight:F1} m)");
            }
            CheckCriticalAltitude();

        }
        public bool CheckCriticalAltitude()
        {
            if (altitude < 100)
            {
                Console.ForegroundColor = ConsoleColor.Red; Console.WriteLine("[CRITICAL] Altitude dropped below safe level!");
                Console.WriteLine(">>> UAV has crashed or was detected by enemy radar. Mission aborted!");
                Console.ResetColor();
                Environment.Exit(0);
                return true;
            }
            return false;
        }
        public void ApplyDamage(double amount, string reason)
        {
            DamageLevel += amount;
            if (DamageLevel > 100) DamageLevel = 100;

            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"[DAMAGE] {reason} → +{amount:F1}% | Total Damage: {DamageLevel:F1}%");
            Console.ResetColor();

            if (DamageLevel >= 100)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[CRITICAL] UAV systems failed due to excessive damage. UAV crashed!");
                Console.ResetColor();
                Environment.Exit(0);
            }
        }
        public void SetReturningToBase()
        {
            if (ReturningBaseCoordinate == null)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine("[ERROR] Return base coordinate is not set.");
                Console.ResetColor();
                return;
            }
            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.WriteLine("Initiating Return to Base...");
            Console.ResetColor();

            Target = ReturningBaseCoordinate;
            IsAltitudeManualControl = true;

            StartMissionSegment();
        }
        // sistem tehdit algıladığında yön saptırması yapacak
        public void PerformMicroEvasiveManeuver()
        {
            double bearing = Position.BrearingTo(Target);
            double addJustToBearing = bearing + (rdr.Next(0, 2) == 0 ? -10 : 10);  //sağa veya sola 10 derece sap

            double offsetLat = 0.00012 * Math.Cos(addJustToBearing * Math.PI / 180);
            double offsetLon = 0.00012 * Math.Sin(addJustToBearing * Math.PI / 180);

            Position.Latitude += offsetLat;
            Position.Longitude += offsetLon;

            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("[ROUTE CORRECTION] Temporary evasive maneuver executed.");
            Console.ResetColor();
        }
        // UnitTest acaba hedef gerçekten vuruluyormu
        public bool IsTargetAchieved(double distanceToTarget)
        {
            return distanceToTarget <= 30;
        }
        // Ana döngü!!
        public void StartMissionSegment()
        {
            Heading = Position.BrearingTo(Target);
            double lastDistanceToTarget = Position.DistanceTo(Target);
            Coordinate startCoordinate = new Coordinate(Position.Latitude, Position.Longitude);
            noflyzone.Clear(); // Sürekli eklenmesin diye sadece 1 kez ekleniyor
            noflyzone.Add(new NoFlyZone(39.9220, 39.9230, 32.8560, 32.8575));
            jammers.Clear();
            jammers.Add(new EnemyJammer(39.9250, 32.8580, 150));
            enemyZones.Clear();
            InitializeEnemyZones();
            friendZones.Clear();
            InitializeFriendZone();
            InitializeRadarZone();
            InitializeNightVision();
            InitalizeStaticObstacle();
            InitializeEMPZones();
            InitializeIrTargets();

            //irtifa
            altitude = GetSimulatedTerrainHeight(Position) + 400;

            Coordinate OrginalTarget = Target;
            bool isInEvasion = false;  // Şu anda kaçış manevrası yapılıyor mu


            for (int i = 0; i < 500; i++)
            {
                wind.UpdateWin();
                CheckForDamage();
                if (!AntiJammingCheck())
                    break;
                CheckObstacleAvoidance();

                // 10 adımda bir rapor ver (sorun yok)
                if (i % 10 == 0)
                {
                    int destroyedCount = AttackTarget.Count(t => t.IsDestroyed);
                    int remainingCount = AttackTarget.Count - destroyedCount;

                    Console.ForegroundColor = ConsoleColor.Magenta;
                    Console.WriteLine($"\n[STATUS REPORT] Destroyed: {destroyedCount}, Remaining: {remainingCount}");
                    Console.ResetColor();
                }
                // Tanımlanan saat aralığından büyük ve eşitse görüş alanının koşul şart ile tanımlanması
                if (nightVisionMode = true && DateTime.Now.Hour >= 20)
                    VisibilityRange = 2500;
                else
                    VisibilityRange = 3000;

                // Sorun yok (düşman bölgesi tespiti) 
                foreach (var zone in enemyZones)
                {
                    if (zone.Center.DistanceTo(Position) < zone.Radius + 30)
                    {
                        string threatKey = $"{Math.Round(zone.Center.Latitude, 4)}|{Math.Round(zone.Center.Longitude, 4)}";
                        if (!KwownThreats.Contains(threatKey))
                        {
                            KwownThreats.Add(threatKey);
                            Console.ForegroundColor = ConsoleColor.DarkMagenta;
                            Console.WriteLine($"[LEARNED] New enemy threat memorized at {threatKey}");
                            Console.ResetColor();
                        }
                    }
                }
                //tehdit varsa 3 ten fazla uçak sapacak    (sorun yok)
                if (IsThreatClustered(Position, 150, 3))   //150 metre içinde 3 tehdit varsa
                {
                    Console.ForegroundColor = ConsoleColor.Magenta;
                    Console.WriteLine("[THREAT ANALYSIS] High threat density detected. Rerouting to avoid cluster...");
                    Console.ResetColor();

                    Coordinate safePoint = GetSafePoint(Position, Target, enemyZones, noflyzone);
                    if (safePoint != null)
                    {
                        Target = safePoint; //Geçici sapma
                        isInEvasion = true;

                        Console.ForegroundColor = ConsoleColor.Yellow;
                        Console.WriteLine("[EVASION] Too many nearby threats! Temporarily changing path...");
                        Console.ResetColor();
                    }
                }
                // Sorun yok
                foreach (var obj in AttackTarget)
                {
                    // ✅ Önce hedef vurulmuşsa hiçbir işlem yapmadan atla
                    if (obj.IsDestroyed)
                        continue;

                    // 🎥 Kamera görüş kontrolü
                    if (!IsTargetVisible(obj.Location))
                    {
                        Console.ForegroundColor = ConsoleColor.DarkGray;
                        Console.WriteLine($"[CAMERA] Target {obj.Type} at ({obj.Location.Latitude:F4}, {obj.Location.Longitude:F4}) not in camera sight. Holding fire.");
                        Console.ResetColor();
                        continue;
                    }

                    // 🎯 Menzil içindeyse saldırı
                    if (Position.DistanceTo(obj.Location) <= 300)
                    {
                        Console.ForegroundColor = ConsoleColor.DarkGreen;
                        Console.WriteLine($"\n[TARGET LOCKED] {obj.Type} at ({obj.Location.Latitude:F6}, {obj.Location.Longitude:F6})");

                        switch (obj.Type)
                        {
                            case "Vehicle":
                                Console.WriteLine("[ENGAGING TARGET] Missile launched. Vehicle destroyed.");
                                break;
                            case "Building":
                                Console.WriteLine("[ENGAGING TARGET] Heavy bomb deployed. Building neutralized.");
                                break;
                            case "Personnel":
                                Console.WriteLine("[ENGAGING TARGET] Smart munition dropped. Personnel neutralized.");
                                break;
                            case "Radar Station":
                                Console.WriteLine("[ENGAGING TARGET] EMP Bomb deployed. Radar station offline.");
                                break;
                            case "Enemy Base":
                                Console.WriteLine("[ENGAGING TARGET] Bunker buster deployed. Enemy base destroyed.");
                                break;
                            case "Weapon Depot":
                                Console.WriteLine("[ENGAGING TARGET] Fuel-air explosive deployed. Weapon depot neutralized.");
                                break;
                            default:
                                Console.WriteLine("[ENGAGING TARGET] Standard bomb deployed. Target neutralized.");
                                break;
                        }
                        // Vurme işlemi tamamlandı
                        Console.ResetColor();
                        obj.IsDestroyed = true;

                        // Öğrenilen tehdit olarak işaretle
                        if (!learnedThereats.Any(t => t.Latitude == obj.Location.Latitude && t.Longitude == obj.Location.Longitude))
                        {
                            learnedThereats.Add(new Coordinate(obj.Location.Latitude, obj.Location.Longitude));
                            Console.ForegroundColor = ConsoleColor.DarkMagenta;
                            Console.WriteLine($"[LEARNED] Previously hostile target memorized at {obj.Location.Latitude:F4}|{obj.Location.Longitude:F4}. Avoiding on return path.");
                            Console.ResetColor();
                        }
                    }
                }
                // düşman gölgesine girildiğinde EMP aktifleşmesi
                bool inEmpZone = empZones.Any(Zone => Zone.IsInZone(Position));

                if (inEmpZone)
                {
                    empActive = true;
                }
                int empTriggerStep = rdr.Next(30, 80);


                if (!IsEMPActive)
                {
                    //IR tespiti
                    foreach (var ir in IRTargets)
                    {
                        if (ir.IsDetected(Position, nightVision.VisibilityRange))
                        {
                            Console.ForegroundColor = ConsoleColor.Red;
                            Console.WriteLine($"IR ALERT: Heat signature detected at ({ir.Locations.Latitude}, {ir.Locations.Longitude})!");
                            Console.ResetColor();
                        }
                    }
                    // radar da bulunma ihtimali
                    bool InRadarZone = radarzone.Any(r => r.IsDetected(Position));
                    if (InRadarZone)
                    {
                        Console.ForegroundColor = ConsoleColor.Yellow;
                        Console.WriteLine("ALERT: Radar detection! Enemy is tracking you.");
                        Console.ResetColor();
                    }
                }
                else
                {
                    empDuractionCounter++;
                    if (empDuractionCounter >= EMP_DURATION_LIMIT)
                    {
                        IsEMPActive = false;
                        Console.ForegroundColor = ConsoleColor.Green;
                        Console.WriteLine("[INFO] EMP effect has worn off. Sensor systems restored.");
                        Console.ResetColor();
                    }
                }
                // Sorun yok
                bool ToCloseToEmeny = enemyZones.Any(z => z.Center.DistanceTo(Position) < z.Radius + 30);

                if (!isInEvasion && ToCloseToEmeny)
                {
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine("[AI] Risk zone detected nearby. Calculating alternative safe route...");
                    Console.ResetColor();

                    // Güvenli bir waypoint bul ve oraya yönel
                    Coordinate safeWayPoint = GetSafePoint(Position, Target, enemyZones, noflyzone);
                    isInEvasion = true;

                    Console.ForegroundColor = ConsoleColor.Cyan;
                    Console.WriteLine("[AUTO ESCAPE] Adjusting route to avoid enemy zone...");
                    Console.ResetColor();

                }
                //sensor hasarı(sorun yok)
                if (IsSensorDamage)
                {
                    sensorFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine($"[WARNING] Sensor damage detected. System retrying... ({sensorFailureCounter}/100)");
                    Console.ResetColor();

                    if (sensorFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] Sensor systems failed. UAV lost targeting and visual capabilities.");
                        Console.ResetColor();
                        return;
                    }
                }
                else sensorFailureCounter = 0;

                //gövde hasarı (sorun yok)
                if (IsFuselageDamage)
                {
                    bodyFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine($"[WARNING] Structural damage increasing. Integrity check ({bodyFailureCounter}/100)");
                    Console.ResetColor();

                    if (bodyFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] UAV structure failed. UAV crashed.");
                        Console.ResetColor();
                        return;
                    }
                }
                else bodyFailureCounter = 0;

                //motor arzası olursa uçuş iptali (sorun yok)
                if (IsEngineDamage && Fuel < 150)
                {
                    engineFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine($"[CRITICAL FAILURE] Engine damaged and fuel low. UAV cannot continue. ({engineFailureCounter}/100)");
                    Console.ResetColor();

                    if (engineFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] Engine damaged and fuel critically low. UAV cannot continue.");
                        Console.ResetColor();
                        return;
                    }
                }
                else
                {
                    engineFailureCounter = 0;  // Eğer durum düzelirse sıfırla
                }

                // Risk kontrolü sonrası (örneğin düşman veya statik engel algılanınca): (sorun yok)
                bool dangerAhead = enemyZones.Any(z => z.Center.DistanceTo(Position) <= z.Radius + 30 ||
                                    statickObstacle.Any(o => o.IsNear(Position)) ||
                                    radarzone.Any(r => r.IsDetected(Position)));

                if (dangerAhead && !IsManuelControl)
                {
                    Console.ForegroundColor = ConsoleColor.Yellow;
                    Console.WriteLine("[AI] Risk zone detected nearby. Calculating alternative safe route...");
                    Console.ResetColor();

                    PerformMicroEvasiveManeuver();
                }
                // EMP etkisi (rastgele aktifleşir)
                if (StepCount > 30 && !IsEMPActive && new Random().NextDouble() < 0.03) // %3 ihtimalle EMP etkisi oluşur
                {
                    IsEMPActive = true;
                    EMPDuration = 5; // 5 adım boyunca kilitlenme süresi
                    Console.ForegroundColor = ConsoleColor.DarkRed;
                    Console.WriteLine($"\n[EMP] Electromagnetic pulse detected! Systems temporarily disabled for {EMPDuration} steps.");
                    Console.ResetColor();
                }

                // EMP etkisi altındaysa
                if (IsEMPActive)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"[EMP EFFECT] System frozen... ({5 - EMPDuration + 1}/5)");
                    Console.ResetColor();

                    EMPDuration--;
                    if (EMPDuration <= 0)
                    {
                        IsEMPActive = false;
                        Console.ForegroundColor = ConsoleColor.Green;
                        Console.WriteLine("[RECOVERY] Systems restored. Resuming mission.");
                        Console.ResetColor();
                    }
                    Thread.Sleep(500); // Görsel olarak beklemesi için (isteğe bağlı)
                    continue; // Döngünün bu adımını atla
                }

                // bağlantı kopması (sorun yok)
                if (!IsConnectionLost && new Random().NextDouble() < 0.03) // %3 ihtimalle Uçak ile merkez arasındaki bağlantı kopar
                {
                    IsConnectionLost = true;
                    ConnectionLostCounter = 0;
                    Console.ForegroundColor = ConsoleColor.DarkRed;
                    Console.WriteLine("[CONNECTION LOST] UAV lost communication. Awaiting reconnection...");
                    Console.ResetColor();
                }
                //bağlantının kopmasıyla birlikte sayaç başlatılır
                if (IsConnectionLost)
                {
                    ConnectionLostCounter++;
                    Console.WriteLine($"[WAITING] Reconnect attempt in progress ({ConnectionLostCounter}/{ConnectionTimeOutLimit})...");

                    // belirtilen limit aşılırsa görev başarısız
                    if (ConnectionLostCounter >= ConnectionTimeOutLimit)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("\n[FAILURE] Reconnection failed. Mission aborted.");
                        Console.ResetColor();
                        return;
                    }
                    // 'R' ye basarsan sistem geri bağlantı sağlar
                    Console.Write("Press 'R' to attempt reconnect: ");
                    while (Console.KeyAvailable)
                    {
                        var key = Console.ReadKey(true);
                        if (key.Key == ConsoleKey.R)
                        {
                            IsConnectionLost = false;
                            Console.ForegroundColor = ConsoleColor.Green;
                            Console.WriteLine("[CONNECTED] UAV communication restored.");
                            Console.ResetColor();
                        }
                    }
                    Thread.Sleep(500);
                    continue;
                }

                //Tuş kontrolü Uçağın irtifasını Ayarlıyor (sorun yok)
                if (!IsManuelControl)
                {
                    CheckCriticalAltitude();
                    while (Console.KeyAvailable)
                    {
                        var key = Console.ReadKey(true);

                        if (key.Key == ConsoleKey.Add || key.Key == ConsoleKey.OemPlus)
                        {
                            altitude += 10;
                            Console.WriteLine($"[Manual] Altitude increased to {altitude:F1} m");
                        }
                        else if (key.Key == ConsoleKey.Subtract || key.Key == ConsoleKey.OemMinus)
                        {
                            altitude -= 10;
                            Console.WriteLine($"[Manual] Altitude decreased to {altitude:F1} m");
                        }
                    }
                }

                else
                {
                    AddJustAltitudeBasedOnTerrain(); // Otomatik irtifa ayarı
                }
                foreach (var ir in IRTargets)
                {
                    if (StepCount > 10 && ir.IsDetected(Position, nightVision.VisibilityRange)) //400 metre menzil
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"IR ALERT: Heat signature detected at ({ir.Locations.Latitude}, {ir.Locations.Longitude})!");
                        ApplyDamage(2.5, "Infrared Target Lot");
                        Console.ResetColor();
                    }
                }
                //Uçak turbulansa girer ise %1 hasar alacak (sorun yok)
                if (wind.Speed > 1.5)
                {
                    if (new Random().NextDouble() < 0.1)
                        ApplyDamage(1.0, "Turbulance");
                }
                //Radar Zone kontrolü
                bool inRadarZone = radarzone.Any(r => r.IsDetected(Position));
               

                //uçağın hasar alması
                if (inRadarZone)
                {
                    ApplyDamage(3.5, "Radar Lock Detected");
                }
                // irtifanın altına düşerse (sorun yok)
                if (altitude < 415)
                {
                    ApplyDamage(5.0, "Too Low Altitude - Possible Terrain Collision");
                }

                // Enemy Zone kontrolü
                bool inEnemyZone = enemyZones.Any(zone => zone.Center.DistanceTo(Position) <= zone.Radius);
                bool inFriendZone = friendZones.Any(zone => zone.IsInsane(Position));

                // --- Enemy Zone Logic --- (sorun yok)
                if (inEnemyZone && !wasInEnemyZone)
                {
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine("[AUTO ESCAPE] Adjusting route to avoid enemy zone...");
                    Console.ResetColor();

                    // 100 metre kuzeye veya güneye kay (basit kaçınma)
                    Position.Latitude += 0.0009;

                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("[ROUTE CORRECTED] Temporary evasive maneuver executed.");
                    Console.ResetColor();

                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("WARNING: Entered ENEMY ZONE! Evade immediately!");
                    Console.ResetColor();
                    IsManuelControl = true;
                    manuelControlCounter = 0;
                    wasInEnemyZone = true;
                }
                else if (!inEnemyZone && wasInEnemyZone)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("Exited ENEMY ZONE. Automatic control restored.");
                    Console.ResetColor();
                    IsManuelControl = false;
                    wasInEnemyZone = false;
                }

                StepCount++;
                // --- Friend Zone Logic --- (sorun yok)
                if (StepCount > 15 && inFriendZone && !wasInFriendZone)
                {

                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("INFO: Entered FRIEND ZONE. You're in a safe area.");
                    Console.ResetColor();
                    wasInFriendZone = true;
                }
                else if (!inFriendZone && wasInFriendZone)
                {
                    wasInFriendZone = false;
                }
                // Restricted alan kontrolü (sorun yok)
                bool nearRestrictedZone = noflyzone.Any(z => z.DistanceTo(Position) < 30.0);
                if (nearRestrictedZone && !IsManuelControl)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("There is a restricted area nearby! You can give directions with the buttons.");
                    Console.ResetColor();
                    IsManuelControl = true;
                    manuelControlCounter = 0;
                }
                
                if (!isInEvasion && Position.DistanceTo(Target) <= 20)
                {
                    Target = OrginalTarget;
                    isInEvasion = true;
                    Console.ForegroundColor = ConsoleColor.Green;
                    //geçiçi kaçınma manevrası gerçekleştirildi
                    Console.WriteLine("[ROUTE CORRECTED] Temporary evasive maneuver executed.");
                    Console.ResetColor();
                }

                double distanceToTarget = Position.DistanceTo(Target);

                // Hedefe ulaşıldı mı? (sorun yok) 
                if (distanceToTarget <= 30)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("\nTarget Successfully Achieved");
                    Console.ResetColor();
                    DrawFlightMap();
                    return;
                }

                // Yakıt kontrolü (sorun yok)
                if (Fuel <= 0)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("\nMission Failed: Fuel exhausted");
                    Console.ResetColor();
                    DrawFlightMap();
                    return;
                }

                // Hedeften uzaklaşıyor mu? (sorun yok)
                if (!IsManuelControl && distanceToTarget > lastDistanceToTarget + 5.0)
                {
                    double diff = distanceToTarget - lastDistanceToTarget;

                    Console.ForegroundColor = ConsoleColor.Yellow;
                    Console.WriteLine("You are moving away from the target. Manual control activated!");
                    Console.ResetColor();

                    // çok fazla saparsa ani uyarı ver
                    if (diff > 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"[CRITICAL DEVIATION] Large deviation detected! UAV veered off course by {diff:F1} meters.");
                        Console.ResetColor();
                    }
                    IsManuelControl = true;
                    manuelControlCounter = 0;
                }

                // Manuel kontrol (sorun yok)
                if (IsManuelControl)
                {
                    while (Console.KeyAvailable)
                    {
                        ConsoleKeyInfo key = Console.ReadKey(true);
                        manuelControlCounter++;

                        switch (key.Key)
                        {
                            case ConsoleKey.UpArrow: Position.Latitude += 0.0001; break;
                            case ConsoleKey.DownArrow: Position.Latitude -= 0.0001; break;
                            case ConsoleKey.LeftArrow: Position.Longitude -= 0.0001; break;
                            case ConsoleKey.RightArrow: Position.Longitude += 0.0001; break;
                        }
                    }
                    if (manuelControlCounter >= manuelControlMaxSteps)
                    {
                        IsManuelControl = false;
                        Console.ForegroundColor = ConsoleColor.Green;
                        Console.WriteLine("Automatic control is restored.");
                        Console.ResetColor();
                    }
                }
                if (Fuel < 70 && !IsReturningToBase)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("[CRITICAL] Fuel low! Returning to base...");
                    IsReturningToBase = true;
                    Console.ResetColor();
                    return;

                }
                // sorun yok
                // radar alanı girdiysen sap yoksa radar emici devreye girer (ama riskli kaçmak en mantıklısı) 
                foreach (var radar in AttackTarget.Where(t => t.Type.ToLower().Contains("radar")))
                {
                    double detectionRange = 600;  //radar menzili
                    double effectiveDistance = Position.DistanceTo(radar.Location) * StealtFactor;

                    if (!radar.IsDestroyed && effectiveDistance <= detectionRange && StepCount > 80)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        //Radar algılandı
                        Console.WriteLine($"[RADAR ALERT] Detected by Radar Station at ({radar.Location.Latitude:F4}, {radar.Location.Longitude:F4}) - Distance: {Position.DistanceTo(radar.Location):F1} m");
                        Console.ResetColor();

                        if (!HasRadarAbsorgingCoating)
                        {
                            // sap 
                            PerformEvasiveManeuver();
                        }

                        else
                        {
                            Console.ForegroundColor = ConsoleColor.DarkMagenta;
                            //radar emici
                            Console.WriteLine("[STEALTH] Stealth coating minimized radar reflection. No lock achieved.");
                            Console.ResetColor();
                        }
                    }
                }
                if (isEvading)
                {
                    // Sap
                    PerformEvasiveManeuver();
                    // kaçınma manevrası azalsın
                    evadeStepCounter--;
                    if (evadeStepCounter <= 0)
                    {
                        isEvading = false;
                        Heading = Position.BrearingTo(Target);  //tekrar hedefe yönel
                        Console.ForegroundColor = ConsoleColor.DarkYellow;
                        // kaçtın şimdi hedefe yönel
                        Console.WriteLine("[INFO] Evasive maneuver complete. Heading re-aligned to target.");
                        Console.ResetColor();
                    }
                }
                else
                {
                    CheckRadarDetectionInSAM();  // radar algılama   (hedef vurulabilir)                  
                    Heading = Position.BrearingTo(Target);  // Hedefe yönel
                }
                // sorun yok 
                foreach (var radar in AttackTarget.Where(t => t.Type.ToLower() == "radar" && !t.IsDestroyed && StepCount > 80))
                {
                    // Tehdit puanı arttırılabilir veya evasive maneuver tetiklenebilir
                    double distance = Position.DistanceTo(radar.Location);
                    if (distance <= radar.DetectionRange)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        // radar uyarısı
                        Console.WriteLine($"[RADAR ALERT] Detected by Radar Station at ({radar.Location.Latitude:F4}, {radar.Location.Longitude:F4}) - Distance: {distance:F1} m");
                        Console.ResetColor();
                    }
                    //Ve menzildeyse artır: (uçağın vurulma ihtimali var)
                    if (distance <= radar.DetectionRange)
                    {
                        radarExposureCounter++;
                        if (radarExposureCounter > 3)
                        {
                            Console.ForegroundColor = ConsoleColor.Red;
                            // uçak vuruldu
                            Console.WriteLine("[WARNING] SİHA hit by enemy SAM! Mission failed.");
                            Console.ResetColor();
                            break;
                        }
                    }
                    else
                    {
                        radarExposureCounter = 0;
                    }
                }
                // Otomatik kontrol (manuel değilse çalışır)
                if (!IsManuelControl)
                {


                    double bearing = Position.BrearingTo(Target);

                    double targetLat = Position.Latitude + 0.00015 * Math.Cos(bearing * Math.PI / 180);
                    double targetLon = Position.Longitude + 0.00015 * Math.Sin(bearing * Math.PI / 180);

                    double errorLat = targetLat - Position.Latitude;
                    double errorLon = targetLon - Position.Longitude;

                    double deltaLat = pidlat.Calculate(errorLat);
                    double deltaLon = pidlon.Calculate(errorLon);

                    bool IsJammed = jammers.Any(j => j.IsInJamRange(Position));
                    double withLat = SensorFusion.ApplyWithEffected(distanceToTarget);
                    double withLon = SensorFusion.ApplyWithEffected(distanceToTarget);

                    double windLat, windLon;
                    double gpsErrorLat, gpsErrorLon;

                    if (IsJammed)
                    {
                        (windLat, windLon) = wind.GettEffect();
                        windLat *= 2.0;
                        windLon *= 2.0;
                        (gpsErrorLat, gpsErrorLon) = SensorFusion.SimulateGPSError(StepCount, DamageLevel + 2);
                    }
                    else
                    {
                        (windLat, windLon) = wind.GettEffect();
                        (gpsErrorLat, gpsErrorLon) = SensorFusion.SimulateGPSError(StepCount, DamageLevel);
                    }

                    Position.Latitude += deltaLat * 0.03 + withLat + windLat + gpsErrorLat;
                    Position.Longitude += deltaLon * 0.03 + withLon + windLon + gpsErrorLon;

                    Fuel -= FuelConsumptionStep + (wind.Speed * 0.03);


                }
                //Sorun yok !!
                // Yasak bölgeye girdi mi?
                foreach (var zone in noflyzone)
                {
                    if (zone.IsInside(Position.Latitude, Position.Longitude))
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"\nEntered the Restricted Area! LAT: {Position.Latitude:F6}, LON: {Position.Longitude:F6}");
                        Console.WriteLine("\nMission cancelled");
                        Console.ResetColor();
                        return;
                    }
                }
                // Uçuş izi kaydı
                FlightPath.Add(new UAVPoint { Lat = Position.Latitude, Lon = Position.Longitude });
                // sorun yok!!!
                Console.ForegroundColor = ConsoleColor.Blue;
                Console.WriteLine($"Step {i + 1:000}: Lat: {Position.Latitude:F6} | Lon: {Position.Longitude:F6} | Target: {distanceToTarget:F2} m | Fuel: {Fuel:F1} | Wind: {wind.Speed:F2} m/s @{wind.Direction:F0}° | Altitude: {altitude:F1} m");
                Console.ResetColor();

                lastDistanceToTarget = distanceToTarget;
                StepCount++;
                Thread.Sleep(1000);
            }
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine("\nMission failed: Objective not achieved.");
            Console.ResetColor();
        }

        public void StartMultiTargetMission(List<Coordinate> targets)
        {
            Coordinate basePoint = new Coordinate(Position.Latitude, Position.Longitude);

            foreach (var target in targets)
            {
                this.Target = target;
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine($"\n>>> New Target Set: LAT {target.Latitude}, LON {target.Longitude}\n");
                Console.ResetColor();

                StartMissionSegment();
                if (Fuel <= 0)
                {
                    Console.WriteLine("Aborting mission due to fuel.");
                    return;
                }
            }
            foreach (var waypoint in MissionWayPoint)
            {
                Target = waypoint;
                StartMissionSegment();
            }
            ReturnToBase(basePoint);
        }
        public void DrawFlightMap()
        {
            int mapWidth = 60;
            int mapHeight = 60;

            if (FlightPath.Count < 2)
                return;

            double minLat = FlightPath.Min(p => p.Lat);
            double maxLat = FlightPath.Max(p => p.Lat);
            double minLon = FlightPath.Min(p => p.Lon);
            double maxLon = FlightPath.Max(p => p.Lon);

            char[,] map = new char[mapHeight, mapWidth];

            for (int y = 0; y < mapHeight; y++)
                for (int x = 0; x < mapWidth; x++)
                    map[y, x] = '.';

            double lonRange = Math.Max(maxLon - minLon, 0.0000001);
            double latRange = Math.Max(maxLat - minLat, 0.0000001);

            // --- Uçuş İzi ---
            for (int i = 0; i < FlightPath.Count; i++)
            {
                var p = FlightPath[i];
                int x = (int)((p.Lon - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((p.Lat - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (i == 0)
                    map[y, x] = 'S'; // Start
                else if (i == FlightPath.Count - 1)
                    map[y, x] = 'T'; // Target
                else
                    if (map[y, x] == '.') map[y, x] = '*';
            }

            // --- Bombalanan hedefler ---
            foreach (var target in AttackTarget.Where(t => t.IsDestroyed))
            {
                int x = (int)((target.Location.Longitude - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((target.Location.Latitude - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (map[y, x] == '.' || map[y, x] == '*')
                    map[y, x] = 'X';
            }

            // --- Öğrenilen tehditler ---
            foreach (var threat in learnedThereats)
            {
                int x = (int)((threat.Longitude - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((threat.Latitude - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (map[y, x] == '.' || map[y, x] == '*')
                    map[y, x] = '!';
            }

            // --- Dost bölge işareti ---
            foreach (var zone in friendZones)
            {
                int x = (int)((zone.Center.Longitude - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((zone.Center.Latitude - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (map[y, x] == '.')
                    map[y, x] = 'F';
            }

            // --- Radar alanları ---
            foreach (var radar in radarzone)
            {
                int x = (int)((radar.Center.Longitude - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((radar.Center.Latitude - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (map[y, x] == '.')
                    map[y, x] = 'R';
            }

            // --- Jammer bölgeleri ---
            foreach (var jammer in jammers)
            {
                int x = (int)((jammer.Center.Longitude - minLon) / lonRange * (mapWidth - 1));
                int y = (int)((jammer.Center.Latitude - minLat) / latRange * (mapHeight - 1));

                if (x < 0) x = 0;
                if (x >= mapWidth) x = mapWidth - 1;
                if (y < 0) y = 0;
                if (y >= mapHeight) y = mapHeight - 1;

                if (map[y, x] == '.')
                    map[y, x] = 'J';
            }

            Console.ForegroundColor = ConsoleColor.DarkGreen;
            Console.WriteLine("\n--- UAV Flight Path Map ---\n");
            Console.ResetColor();

            for (int y = 0; y < mapHeight; y++)
            {
                for (int x = 0; x < mapWidth; x++)
                    Console.Write(map[y, x]);
                Console.WriteLine();
            }
        }
        public void ReturnToBase(Coordinate basePoint)
        {
            Console.ForegroundColor = ConsoleColor.Cyan;
            Console.WriteLine("\nInitiating Return to Base...");
            Console.ResetColor();

            PIDController pidlat = new PIDController(0.2, 0.5, 0.1);
            PIDController pidlot = new PIDController(0.2, 0.5, 0.1);

            double LastDistance = Position.DistanceTo(basePoint);

            noflyzone.Clear(); // Sürekli eklenmesin diye sadece 1 kez ekleniyor
            jammers.Clear();
            jammers.Add(new EnemyJammer(39.9250, 32.8580, 150));
            enemyZones.Clear();
            InitializeEnemyZones();
            friendZones.Clear();
            InitializeFriendZone();
            InitializeRadarZone();
            InitializeNightVision();
            InitalizeStaticObstacle();
            CheckObstacleAvoidance();
            altitude = GetSimulatedTerrainHeight(Position) + 400;
            bool isInEvasion = false;  // Şu anda kaçış manevrası yapılıyor mu

            while (true)
            {
                wind.UpdateWin();
                CheckForDamage();
                if (!AntiJammingCheck())
                    break;

                bool ToCloseToEmeny = enemyZones.Any(z => z.Center.DistanceTo(Position) < z.Radius + 30);

                if (!isInEvasion && ToCloseToEmeny)
                {
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine("[AI] Risk zone detected nearby. Calculating alternative safe route...");
                    Console.ResetColor();

                    // Güvenli bir waypoint bul ve oraya yönel
                    Coordinate safeWayPoint = GetSafePoint(Position, Target, enemyZones, noflyzone);
                    isInEvasion = true;

                    Console.ForegroundColor = ConsoleColor.Cyan;
                    Console.WriteLine("[AUTO ESCAPE] Adjusting route to avoid enemy zone...");
                    Console.ResetColor();

                }

                // KnownThreats avoidance logic.
                //RTB aktifken, her adımda tehdit hafızasındaki noktalara çok yakınsa kaçınma yapılacak.
                if (IsReturningToBase)
                {
                    foreach (var threat in KwownThreats)
                    {
                        var parts = threat.Split('|');
                        double lat = double.Parse(parts[0]);
                        double lon = double.Parse(parts[1]);
                        Coordinate threatCoord = new Coordinate(lat, lon);

                        double dist = Position.DistanceTo(threatCoord);

                        // 80 metre tehlike yarıçapı
                        if (dist < 80)
                        {
                            Console.ForegroundColor = ConsoleColor.DarkYellow;
                            Console.WriteLine($"[RTB AVOIDANCE] Known threat at {threat} too close ({dist:F1} m). Adjusting course...");
                            Console.ResetColor();

                            // Kaçınma manevrası – doğrudan biraz kuzeye kay
                            Position.Latitude += 0.00025;

                            // Hedef yeniden setleniyor (base aynı kalsın ama yönü düzelt)
                            break;
                        }
                    }
                }

                // Gövde (Fuselage) gövde hasarı
                if (IsSensorDamage)
                {
                    sensorFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine($"[WARNING] Sensor damage detected. System retrying... ({sensorFailureCounter}/100)");
                    Console.ResetColor();

                    if (sensorFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] Sensor systems failed. UAV lost targeting and visual capabilities.");
                        Console.ResetColor();
                        return;
                    }
                }
                else sensorFailureCounter = 0;

                if (IsFuselageDamage)
                {
                    bodyFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine($"[WARNING] Structural damage increasing. Integrity check ({bodyFailureCounter}/100)");
                    Console.ResetColor();

                    if (bodyFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] UAV structure failed. UAV crashed.");
                        Console.ResetColor();
                        return;
                    }
                }
                else bodyFailureCounter = 0;
                //motor arzası olursa uçuş iptali
                if (IsEngineDamage && Fuel < 50)
                {
                    engineFailureCounter++;
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("[CRITICAL FAILURE] Engine damaged and fuel low. UAV cannot continue.");
                    Console.ResetColor();

                    if (engineFailureCounter >= 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("[CRITICAL FAILURE] Engine damaged and fuel critically low. UAV cannot continue.");
                        Console.ResetColor();
                        return;
                    }
                }
                else
                {
                    engineFailureCounter = 0;  // Eğer durum düzelirse sıfırla
                }


                if (!IsConnectionLost && new Random().NextDouble() < 0.03)
                {
                    IsConnectionLost = true;
                    ConnectionLostCounter = 0;
                    Console.ForegroundColor = ConsoleColor.DarkRed;
                    Console.WriteLine("[CONNECTION LOST] UAV lost communication. Awaiting reconnection...");
                    Console.ResetColor();
                }
                if (IsConnectionLost)
                {
                    ConnectionLostCounter++;
                    Console.WriteLine($"[WAITING] Reconnect attempt in progress ({ConnectionLostCounter}/{ConnectionTimeOutLimit})...");

                    if (ConnectionLostCounter >= ConnectionTimeOutLimit)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine("\n[FAILURE] Reconnection failed. Mission aborted.");
                        Console.ResetColor();
                        return;
                    }
                    Console.Write("Press 'R' to attempt reconnect: ");
                    while (Console.KeyAvailable)
                    {
                        var key = Console.ReadKey(true);
                        if (key.Key == ConsoleKey.R)
                        {
                            IsConnectionLost = false;
                            Console.ForegroundColor = ConsoleColor.Green;
                            Console.WriteLine("[CONNECTED] UAV communication restored.");
                            Console.ResetColor();
                        }
                    }
                    Thread.Sleep(500);
                    continue;
                }

                if (!IsManuelControl)
                {
                    CheckCriticalAltitude();
                    while (Console.KeyAvailable)
                    {
                        var key = Console.ReadKey(true);

                        if (key.Key == ConsoleKey.Add || key.Key == ConsoleKey.OemPlus)
                        {
                            altitude += 10;
                            Console.WriteLine($"[Manual] Altitude increased to {altitude:F1} m");
                        }
                        else if (key.Key == ConsoleKey.Subtract || key.Key == ConsoleKey.OemMinus)
                        {
                            altitude -= 10;
                            Console.WriteLine($"[Manual] Altitude decreased to {altitude:F1} m");
                        }
                    }
                }

                else
                {
                    AddJustAltitudeBasedOnTerrain(); // Otomatik irtifa ayarı
                }

                foreach (var ir in IRTargets)
                {
                    if (ir.IsDetected(Position, nightVision.VisibilityRange)) //400 metre menzil
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"IR ALERT: Heat signature detected at ({ir.Locations.Latitude}, {ir.Locations.Longitude})!");
                        Console.ResetColor();
                    }
                }

                bool InRadarZone = radarzone.Any(r => r.IsDetected(Position));
                if (InRadarZone)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("ALERT: Radar detection! Enemy is tracking you.");
                    Console.ResetColor();
                }
                //uçağın hasar alması
                if (InRadarZone)
                {
                    ApplyDamage(3.5, "Radar Lock Detected");
                }
                // irtifanın altına düşerse
                if (altitude < 415)
                {
                    ApplyDamage(5.0, "Too Low Altitude - Possible Terrain Collision");
                }

                if (Fuel <= 0)
                {
                    Console.ForegroundColor = ConsoleColor.DarkRed;
                    Console.WriteLine("\nMission Failed: Fuel exhausted on return!");
                    Console.ResetColor();
                    DrawFlightMap();
                    return;
                }

                double CurrentDistance = Position.DistanceTo(basePoint);
                if (CurrentDistance <= 30.0)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("\nReturn To Base Successfuly");
                    Console.ResetColor();
                    DrawFlightMap();
                    return;
                }
                if (!IsManuelControl && CurrentDistance > LastDistance + 0.05)
                {
                    double diff = CurrentDistance - LastDistance;

                    Console.ForegroundColor = ConsoleColor.Yellow;
                    Console.WriteLine("You are moving away from the base. Manual control activated!");
                    Console.ResetColor();

                    if (diff > 100)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"[CRITICAL DEVIATION] Large deviation detected! UAV veered off course by {diff:F1} meters.");
                    }

                    IsManuelControl = true;
                    manuelControlCounter = 0;

                }
                if (IsManuelControl)
                {
                    while (Console.KeyAvailable)
                    {
                        ConsoleKeyInfo key = Console.ReadKey(true);
                        manuelControlCounter++;
                        switch (key.Key)
                        {
                            case ConsoleKey.UpArrow:
                                Position.Latitude += 0.0001;
                                break;
                            case ConsoleKey.DownArrow:
                                Position.Latitude -= 0.0001;
                                break;
                            case ConsoleKey.LeftArrow:
                                Position.Longitude -= 0.0001;
                                break;
                            case ConsoleKey.RightArrow:
                                Position.Longitude += 0.0001;
                                break;
                        }
                    }
                }
                if (IsManuelControl && manuelControlCounter >= manuelControlMaxSteps)
                {
                    IsManuelControl = false;
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("Automatic control is restored.");
                    Console.ResetColor();
                }
                bool inEnemyZone = enemyZones.Any(zone => zone.Center.DistanceTo(Position) <= zone.Radius);

                if (inEnemyZone && !wasInEnemyZone)
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("WARNING: Entered ENEMY ZONE! Evade immediately!");
                    Console.ResetColor();
                    IsManuelControl = true;
                    manuelControlCounter = 0;
                    wasInEnemyZone = true;
                }
                else if (!inEnemyZone && wasInEnemyZone)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("Exited ENEMY ZONE. Automatic control restored.");
                    Console.ResetColor();
                    IsManuelControl = false;
                    wasInEnemyZone = false;
                }
                bool inFriendZone = friendZones.Any(zone => zone.IsInsane(Position));

                if (inFriendZone && !inEnemyZone && !wasInFriendZone)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("INFO: Entered FRIEND ZONE. You're in a safe area.");
                    Console.ResetColor();
                    wasInFriendZone = true;
                }
                else if (!inFriendZone && wasInFriendZone)
                {
                    wasInFriendZone = false;
                }
                if (IsManuelControl)
                {
                    while (Console.KeyAvailable)
                    {
                        ConsoleKeyInfo key = Console.ReadKey(true);
                        switch (key.Key)
                        {
                            case ConsoleKey.UpArrow:
                                Position.Latitude += 0.0001;
                                break;
                            case ConsoleKey.DownArrow:
                                Position.Latitude -= 0.0001;
                                break;
                            case ConsoleKey.LeftArrow:
                                Position.Longitude -= 0.0001;
                                break;
                            case ConsoleKey.RightArrow:
                                Position.Longitude += 0.0001;
                                break;
                        }
                        manuelControlCounter++;
                    }
                }

                if (IsManuelControl && manuelControlCounter >= manuelControlMaxSteps)
                {
                    IsManuelControl = false;
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("Automatic control is restored.");
                    Console.ResetColor();
                }
                CheckCriticalAltitude();
                double bearing = Position.BrearingTo(basePoint);

                double TargetLat = Position.Latitude + 0.00015 * Math.Cos(bearing * Math.PI / 180);
                double TargetLon = Position.Longitude + 0.00015 * Math.Sin(bearing * Math.PI / 180);

                double ErrorLat = TargetLat - Position.Latitude;
                double ErrorLon = TargetLon - Position.Longitude;

                double deltaLat = pidlat.Calculate(ErrorLat);
                double deltaLon = pidlon.Calculate(ErrorLon);

                bool IsJammed = jammers.Any(j => j.IsInJamRange(Position));
                double withLat = SensorFusion.ApplyWithEffected(CurrentDistance);
                double WithLon = SensorFusion.ApplyWithEffected(CurrentDistance);

                double windlat, windlon;
                double GpsErrorLat, GpsErrorLon;

                if (IsJammed)
                {
                    Console.ForegroundColor = ConsoleColor.DarkYellow;
                    Console.WriteLine("[JAMMING DETECTED] Navigation interference active!");
                    Console.ResetColor();

                    (windlat, windlon) = wind.GettEffect();
                    windlat *= 2.0;
                    windlon *= 2.0;
                    (GpsErrorLat, GpsErrorLon) = SensorFusion.SimulateGPSError(StepCount, DamageLevel + 2);
                }
                else
                {
                    (windlat, windlon) = wind.GettEffect();
                    (GpsErrorLat, GpsErrorLon) = SensorFusion.SimulateGPSError(StepCount, DamageLevel);
                }

                Position.Latitude += deltaLat * 0.03 + withLat + windlat + GpsErrorLat;
                Position.Longitude += deltaLon * 0.03 + WithLon + WithLon + GpsErrorLon;

                Fuel -= FuelConsumptionStep + (wind.Speed * 0.03);

                Console.ForegroundColor = ConsoleColor.Blue;
                Console.WriteLine($"Returning... Lat: {Position.Latitude:F6} | Lon: {Position.Longitude:F6} | To Base: {CurrentDistance:F2} m | Fuel: {Fuel:F1} | Wind: {wind.Speed:F2} m/s @{wind.Direction:F0}° | Altitude: {altitude:F1} m");
                Console.ResetColor();

                LastDistance = CurrentDistance;

                Thread.Sleep(1000);
            }
        }
               
    }
    class Wind
    {
        private static Random rand = new Random();
        public double Direction { get; private set; }
        public double Speed { get; private set; }

        public Wind()
        {
            Direction = rand.Next(0, 360);    // 0–359 derece arasında yön
            Speed = rand.NextDouble() * 2;   // 0-2 m/s arası hafif rüzgar   // 0.0–1.0 arası hız
        }
        public (double,double) GettEffect()
        {
            //Bu iki hesaplama, harita üzerinde hareket eden bir nesnenin küçük bir zaman diliminde (örneğin 1 adımda) enlem ve boylam yönünde ne kadar ilerleyeceğini belirler.
            //Math.Cos(...): Bu yönün kuzey-güney (enlem) bileşenini hesaplar.
            double effectLat = Speed * 0.00001 * Math.Cos(Direction * Math.PI / 180);
            double effectLon = Speed * 0.00001 * Math.Sin(Direction * Math.PI / 180);
            return (effectLat, effectLon);
        }
        // rüzgarın uçağa etkisi
        public void UpdateWin()
        {           
            Direction += rand.NextDouble() * 4 - 2;           
            Speed += rand.NextDouble() * 0.2 - 0.1;           
            Speed = Math.Max(0.0, Math.Min(5.0, Speed));
        }
    }
    public class UAVPoint
    {
        public double Lat { get; set; }
        public double Lon { get; set; }
    }
    public class NoFlyZone
    {
        public double MinLat { get; set; }
        public double MaxLat { get; set; }
        public double MinLon { get; set; }
        public double MaxLon { get; set; }
        public NoFlyZone(double minLat, double maxLat, double minLon, double maxLon)
        {
            MinLat = minLat;
            MaxLat = maxLat;
            MinLon = minLon;
            MaxLon = maxLon;
        }
        public bool IsInside(double lat, double lon)
        {
            return lat >= MinLat && lat <= MaxLat && lon >= MinLon && lon <= MaxLon;
        }
        //
        public double DistanceTo(Coordinate point)
        {        
            double lat = Math.Max(MinLat, Math.Min(point.Latitude, MaxLat));
            double lon = Math.Max(MinLon, Math.Min(point.Longitude, MaxLon));

            Coordinate nearest = new Coordinate(lat, lon);
            return point.DistanceTo(nearest);
        }
    }
    class RadarZone
    {
        public Coordinate Center { get; set; }
        public double Radius { get; set; }

        public RadarZone(double lat, double lon, double radius)
        {
            Center = new Coordinate(lat, lon);
            Radius = radius;
        }
        public bool IsDetected(Coordinate position)
        {
            return Center.DistanceTo(position) <= Radius;
        }
    }
    class IRTarget
    {
        public Coordinate Locations { get; set; }
        public double HeatSignature { get; set; }

        public IRTarget(double lat, double lon, double heat)
        {
            Locations = new Coordinate(lat, lon);
            HeatSignature = heat;
        }
        //
        public bool IsDetected(Coordinate uavPosition, double detectionRadius)
        {
            return Locations.DistanceTo(uavPosition) <= detectionRadius && HeatSignature >= 0.6;
        }
    }
    class NightVision
    {
        public bool IsNightTime { get; set; }
        public double Temperature { get; set; } // °C cinsinden
        public double VisibilityRange { get; private set; } //km cinsinden

        public NightVision(bool isNightTime, double temperature)
        {
            IsNightTime = isNightTime;
            Temperature = temperature;
            UpdateVisibility();
        }
        public void UpdateVisibility()
        {
            if (IsNightTime)
            {
                if (Temperature < 5)
                    VisibilityRange = 0.8;
                else if (Temperature <= 15)
                    VisibilityRange = 1.2;
                else
                    VisibilityRange = 1.5;
            }
            else
            {
                VisibilityRange = 4.0;
            }
        }
        public void PrintStatus()
        {
            Console.ForegroundColor = ConsoleColor.DarkCyan;
            Console.WriteLine($"[Night Vision] Mode: {(IsNightTime ? "Night" : "Day")}, Temp: {Temperature}°C, Visibility: {VisibilityRange} km");
            Console.ResetColor();
        }
    }
    class LidarSensor
    {
        private Random rand = new Random();

        // Simüle edilmiş yüzey yüksekliği (deniz seviyesinden)
        public double GetSurfaceAltitude(Coordinate coord)
        {
            // Örnek: 900m - 1300m arası bir yükseklik verisi simüle edilir
            return 900 + rand.NextDouble() * 400;
        }
    }
     class EnemyJammer
    {
        public Coordinate Center { get; set; }
        public double Radius { get; set; }

        public EnemyJammer(double lat, double lon, double radius)
        {
            Center = new Coordinate(lat, lon);
            Radius = radius;
        }
        public bool IsInJamRange(Coordinate pos)
        {
            return Center.DistanceTo(pos) <= Radius;
        }
    }
   public class EMPZone
    {
        public Coordinate Center { get; set; }
        public double Radius { get; set; } = 1000;

        public bool IsInZone(Coordinate position)
        {
            return Center.DistanceTo(position) <= Radius;
        }
        public EMPZone(double lat, double lon, double radius)
        {
            Center = new Coordinate(lat, lon);
            Radius = radius;
        }

    }
   public  class TargetObject
    {
        public double Latitude { get; set; }
        public double Longitude { get; set; }
        public Coordinate Location { get; set; }
        public bool IsDestroyed { get; set; } = false;
        public string Type { get; set; } //ör Radar / Base / conwoy
        public int Priority { get; set; }
        public double DetectionRange { get; set; } = 500;  //metre cinsinden
        public TargetObject(double lat, double lon, string type)
        {
            Latitude = lat;
            Longitude = lon;
            Type = type;
            IsDestroyed = false;
            Location = new Coordinate(lat, lon);
            Priority = GetPriorityByType(type);

            if (type.ToLower() == "radar station")
            {
                DetectionRange = 600; //daha geniş menzil
            }
        }
        // puan katsayısına göre bombardıman yapılacak hedefler öncelenir
        private int GetPriorityByType(string type)
        {
            switch (type.ToLower())
            {
                case "radar station": return 90;
                case "personnel": return 40;
                case "vehicle": return 50;
                case "building": return 30;
                case "weapon depot": return 80;
                case "enemy base": return 95;
                default: return 10;
            }
        }
        public Coordinate GetCoordinate()
        {
            return new Coordinate(Latitude, Longitude);
        }
    }
   public class Obstacle
    {
        public Coordinate Location { get; set; }
        public double Radius { get; set; } //etki alanı yarıçapı (metre)
        public double Height { get; set; } //Engel yüksekliği (metre)
        public Obstacle(double lat, double lon, double height, double radius)
        {
            Location = new Coordinate(lat, lon);
            Height = height;
            Radius = radius;
        }
        public bool IsNear(Coordinate position)
        {
            return Location.DistanceTo(position) <= Radius;
        }
    }
    public enum WeanponType
    {
        Bomb,
        Missile,
        SmartBomb
    }
   public class WeaponSystem
    {
        private UAV uav;
        public WeaponSystem(UAV uavInstance)
        {
            this.uav = uavInstance;
        }
        public void Engage(TargetObject target)
        {
            if (target.IsDestroyed) return;

            double distance = uav.Position.DistanceTo(target.GetCoordinate());

            if (distance <= 1500)
            {
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine($"[ENGAGE] {target.Type} within range ({distance:F2} m).");
                Console.ResetColor();

                if (target.Type == "Radar Station" || target.Type == "Enemy Base")
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.WriteLine("[BOMB DROP] Bomb released over target!");
                    Console.ResetColor();
                }
                else
                {
                    Console.ForegroundColor = ConsoleColor.Red;
                    Console.WriteLine("[MISSILE LAUNCH] Missile launched at target!");
                    Console.ResetColor();
                }
                target.IsDestroyed = true;
                Console.ForegroundColor = ConsoleColor.Green;
                Console.WriteLine($"[HIT CONFIRMED] {target.Type} has been neutralized.");
                Console.ResetColor();
            }
        }
    }
    
    class PIDController
    {
        public double kp, ki, kd;
        private double integral = 0;
        private double previousError = 0;

        public PIDController(double kp, double ki, double kd)
        {
           
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }
        public double Calculate(double error)
        {           
            integral += error;
            
            double derivative = error - previousError;
            //Gelecek adımda derivative hesaplaması için bugünkü hatayı kaydeder.
            previousError = error;
            
            return kp * error + ki * integral + kd * derivative;
        }
        static void Main(string[] args)
        {

            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine("--- SİHA Flight Simulation ---\n");
            Console.ResetColor();
           UAV uav = new UAV(
                new Coordinate(39.920770, 32.854110), //Start
                null
                );
            uav.AttackTarget.Add(new TargetObject(39.9250, 32.8550, "Vehicle"));
            uav.AttackTarget.Add(new TargetObject(39.9260, 32.8560, "Personnel"));
            uav.AttackTarget.Add(new TargetObject(39.9270, 32.8570, "Radar Station"));
            uav.AttackTarget.Add(new TargetObject(39.9280, 32.8580, "Building"));
            uav.AttackTarget.Add(new TargetObject(39.9282, 32.8608, "Enemy Base"));
            uav.AttackTarget.Add(new TargetObject(39.2880, 32.8605, "Weapon Depot"));
            uav.AttackTarget.Add(new TargetObject(39.9278, 32.8662, "Radar Station"));
            uav.AttackTarget.Add(new TargetObject(39.9282, 32.8608, "Enemy Base"));

            uav.MissionWayPoint.Add(new Coordinate(39.9250, 32.8550)); // Waypoint 1 Vehicle
            uav.MissionWayPoint.Add(new Coordinate(39.9260, 32.8560)); // Waypoint 2 personel
            uav.MissionWayPoint.Add(new Coordinate(39.9270, 32.8570)); // Waypoint 3 radar station
            uav.MissionWayPoint.Add(new Coordinate(39.9280, 32.8580)); // Waypoint 4 Building
            uav.MissionWayPoint.Add(new Coordinate(39.9282, 32.8608)); // Waypoint 5 Enemy Base
            uav.MissionWayPoint.Add(new Coordinate(39.9278, 32.8662)); // Waypoint 6

           
            uav.HasRadarAbsorgingCoating = true;
            WeaponSystem weaponSystem = new WeaponSystem(uav);


            List<Coordinate> missionTarget = new List<Coordinate>
            {
                new Coordinate(39.92600, 32.86400),
                new Coordinate(39.92750, 32.86700),
                new Coordinate(39.92880, 32.86050),
            };
            Console.ForegroundColor = ConsoleColor.Yellow;
            Console.WriteLine("--- Starting UAV Flight ---\n");
            Console.ResetColor();
            Console.Write("press any key ");
            var mode = Console.ReadLine()?.ToUpper();

            if (mode == "")
                uav.IsAltitudeManualControl = true;
            else
                uav.IsAltitudeManualControl = false;

            uav.StartMultiTargetMission(missionTarget);
            Console.ForegroundColor = ConsoleColor.DarkRed;
            Console.WriteLine("\nTask completed. Press any key to exit.");
            Console.ResetColor();
            Console.ReadKey();
        }
    }
   
    class FriendZone
    {
        public Coordinate Center { get; set; }
        public double Radius { get; set; }

        public bool IsInsane(Coordinate position)
        {
            return position.DistanceTo(Center) <= Radius; 
        }
    }
   public  class EnemyZone
    {
        public Coordinate Center { get; set; }
        public double Radius { get; set; }
        
    }
    class SensorFusion
    {
        private static Random rand = new Random();
        public static double ApplyWithEffected(double distanceToTarget)
        {           
            double scale = Math.Min(distanceToTarget / 100.0, 1.0); // 100 m'den sonra tam etki, daha yakınsa azalt
        
            return (rand.NextDouble() * 0.0001 - 0.00005) * scale;
        }
        public static (double LatError, double LonError) SimulateGPSError(int step, double damageLevel)
        {
            if (step %15 == 0)
            {
                //±0.00025 derece (~±25-30 metre) civarında sapma uygular.
                //Bu sapma genellikle “ani konum hatası” gibi hissedilir.
                double errorLat = (rand.NextDouble() - 0.5) * 0.0005;
                double errorLon = (rand.NextDouble() - 0.5) * 0.0005;
                double errorRange = 0.00005 + (damageLevel / 10000); // %50 hasarda ~0.0001 sapma
                Random rnd = new Random();
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine("GPS Deviation Occurred! (+{0:F6}, +{1:F6})", errorLat, errorLon);
                Console.ResetColor();
                return (rnd.NextDouble() * errorRange, rnd.NextDouble() * errorRange);
            }
            return (0, 0);
        }
    }   
}
