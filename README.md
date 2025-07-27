Aircraft Software System
The aircraft's takeoff coordinates (latitude and longitude) are 39.920770, 32.854110)
The 3 coordinates it must go to, respectively ==> 39.92600, 32.86400, 39.92750, 32.86700, 39.92880, 32.86050
and the 6 targets it must hit between the defined coordinates
(39.9250, 32.8550)); // Waypoint 1 Vehicle
(39.9260, 32.8560)); // Waypoint 2 Personnel
(39.9270, 32.8570)); // Waypoint 3 Radar Station
(39.9280, 32.8580)); // Waypoint 4 Building
(39.9282, 32.8608)); // Waypoint 5 Enemy Base
(39.9278, 32.8662)); // Waypoint 6 Radar station

1- Problems the plane will encounter along its path
- Connection loss: Press 'R' or the connection with the plane will be lost and the mission will be canceled.
- System locking: You cannot make any changes to the aircraft for 5 steps. The system is locked (maximum 5 steps).
- The aircraft travels to its destination within the defined coordinates, but external factors (such as wind drift)
(encountering obstacles (buildings, mountains))
automatically increase its altitude (you can also set it manually).
Note: The aircraft may enter turbulence and suffer a 1% chance of damage. If your altitude drops below 415 meters, this will cause 5% damage.
The aircraft automatically adjusts its altitude when necessary (automatic control).
You can adjust the aircraft's altitude by pressing the '+' and '-' buttons.

The aircraft has a less than 1 in 1000 chance of sustaining body damage and may become unusable after 100 steps.
The aircraft has a less than 2 in 1000 chance of sustaining engine damage (this shouldn't be a problem if your fuel level is above a certain level).
The aircraft has a less than 3 in 1000 chance of sustaining sensor damage. It becomes unusable after 100 steps.

Jamming: There is a 5% chance that if this happens for 3 consecutive steps, the aircraft will become unusable.

(However, if it doesn't happen 3 times in a row, the counter will reset, allowing the aircraft to continue.)

- Enemy coordinates
Coodinate = (39.9240, 32.8600),
Radius = 300.0
If you enter this, it will give a warning and disable auto-control, allowing you to manually control the aircraft.

You can adjust the aircraft's latitude and longitude (coordinates) to quickly exit the area.
!!! If you are detected, the mission will be aborted and you will return to base.

CRITICAL POINT:

radarzone.Add(new RadarZone(39.9240, 32.8570, 0.3)); // 300-meter-radius radar zone
radarzone.Add(new RadarZone(39.9070, 32.8640, 0.25)); // another radar zone

As you approach these zones (average 40 steps), you'll receive a warning.
- Here, the AI automatically activates, and the aircraft's radar absorbers reduce the chance of detection (70% success rate).
- Depending on the situation, the AI automatically calculates a suitable route and provides a deviation.
- It then returns to the target.
- The aircraft can be detected by the radar and shot down.
- All targets to be hit proceed in a specific order (it may not hit three targets while en route to the first target, but the coordinates already known are recorded, and the location of the third target is clearly determined).
- The coordinates of the areas you hit are recorded. Before the hit, the camera is in the field of view. The fire is fired, and the hit report is output. - The areas hit on the camera are now disabled, and only the remaining enemy coordinates are listed for subsequent steps.
- The aircraft's path is displayed every 1 second through defined loops (Target).

When the aircraft reaches a target at the coordinates you are traveling to and then redirects to another target, the aircraft's path is output.

- If it enters enemy territory or deviates due to wind, manual control is activated. You must set the latitude and longitude (coordinates) yourself using the keys.

- You may not be able to hit all targets. You may miss a few of the six, and the aircraft will return to base depending on fuel levels and other factors.

You can pass through enemy territory again while returning to base.

If all goes well, if you don't hit all targets but only hit a few, the mission is partially successful. If you return to base without any problems at 39.920770 or 32.854110, the mission is completed.





Uçak Yazılım sistemi
Uçağın kalkış yapacağı koordinatlar (enlem ve boylam) 39.920770, 32.854110)
Sırasıyla gitmesi gereken 3 koordinat  ==> 39.92600, 32.86400,  39.92750, 32.86700,   39.92880, 32.86050
ve tanımlanan koordinatlar arasında vurması gereken 6 hedef
(39.9250, 32.8550)); // Waypoint 1 Vehicle
(39.9260, 32.8560)); // Waypoint 2 personel
(39.9270, 32.8570)); // Waypoint 3 radar station
(39.9280, 32.8580)); // Waypoint 4 Building
(39.9282, 32.8608)); // Waypoint 5 Enemy Base
(39.9278, 32.8662)); // Waypoint 6 Radar station

1- uçağın aldığı yol boyunca karşılaşacağı sorunlar
- Bağlantı kopması 'R' ye basmanız gerek yoksa uçak ile bağlantı kopar ve görev iptal olur.
- Sistemin kitlenmesi 5 adım boyunca uçağa hiç bir müdahale yapamazsınız sistem kitlenir (en fazla 5 adım)
- Uçak gitmesi gereken hedefe tanımlanan koordinatlar aralığında gider ama dış faktörler
(Rüzgar yönünü saptırır)
(engellerle karşılaşır (bina, dağ))
otomatik olarak irtfasını arttırır (manuel olarakta ayarlayabilirsiniz)
Not: Uçak türbülansa girebilir ve %1 hasar alma ihtimali vardır ve irtifanız 415 metrenin altına düşerse bu uçağa %5 lik bir hasar verir
uçak gerektiği yerde irtifasını otomatik olarak kendi ayarlar (oto kontrol)
Uçağın irtifasını '+' ve '-' ye basarak ayarlayabilirsiniz

uçak  1000/1 den küçük bir ihtimalle gövde hasarı alabilir ve 100 step sonra uçak kullanılamaz hale gelebilir
uçak  1000/2 den küçük bir ihtimalle motor hasarı alabilir (bu çok sorun yaratmaz yakıtınız belli bir seviyenin üzerindeyse)
uçak  1000/3 den küçük bir ihtimalle sensor hasarı alabilir 100 adım sonra kullanışsız bir hale gelir
-

Jamming: %5 ihtimal vardır bu ihtimal in 3 step boyunca ard arda olursa uçak kullanılamaz hale gelir
(ama 3 defa ard arda olmassa sayaç başa sarar buda uçağın devam etmesine olanak sağlar)

-
düşman koordinatları
Coodinate = (39.9240, 32.8600),
Radius = 300.0
Eğer girerseniz uyarı verir ve oto kontrol devre dışı kalır buda manuel olarak kontrol etmenize olanak sağlar böylece 
uçağın enlem ve boylamını (koordinatlarını) ayarlayarak hızlıca bölgeden çıkmasına olanak sağlarsınız
!!! eğer Tespit edilirseniz görev iptal olur ve base e dönersiniz
-

KRİTİK NOKTA: 

radarzone.Add(new RadarZone(39.9240, 32.8570, 0.3)); // 300 metre yarıçaplı radar alanı
radarzone.Add(new RadarZone(39.9070, 32.8640, 0.25)); // başka bir radar alanı

bu bölgelere yaklaştıkça (ortalama 40. adımdan)  sonra uyarı alırsınız
- Burda yapay zeka otomatik devreye girer ve uçağın radar emicileri  tespit edilme ihtimalini azaltır (%70 başarı ihtimali).
- Duruma göre yapay zeka oto kontrol ile uygun bir rota hesaplar ve sapma sağlanır.
- Daha sonrasında tekrar hedefe yönelir. 
- Uçak radar tarafından tespit edilip düşerülebilir.
- Vurulması gereken bütün hedefler in belli bir sıraya göre ilerler (1. hedefe gidiyorken 3 hedefin yanından geçerken vurmaz ama zaten halı hazırda belli olan kordinatlar kayda alınır ve 3. hedefin yeri net bir şekilde tespit edilir)
- vurduğunuz alanların kordinatları kayda geçer vurmadan önce kamera görüş alanındadır ateşleme olunur ve vurulur bunun raporu çıktı olarak verilir.
- Kamerada vurulan bölgeler artık devre dışıdır ve diğer adımlarda sadece vurulması için kalan düşman koordinatlarını listeler
- Uçağın kattettiği  yol tanımlanan döngüler aracılığıyla her 1 saniyede gösterilir (Target)

Uçak gittiğiniz koordinatlarda bir hedefe varıp başka bir hedefe yönlendiğinde uçağın aldığı yolun çıktısı verilir

-

düşman bölgesine girer veya rüzgarında etkisiyle uçak sapar ise manuel kontrol devreye girer enlem ve boylamını (koordinatlarını) sizin ayarlamanız gerekir tuşlar aracılığıyla



- bütün hedefleri vuramayabilirsiniz 6 taneden birkaçını es geçebilir uçak yakıt durumuna ve diğer etkenlere göre base ' e geri döner
base e dönerken tekrardan düşman bölgelerinde geçebilirsiniz

herşey yolunda giderse tüm hedefleri vuramayıp sadece bir kaç hedefi dahi vurursanız görev kısman başarılıdır eğer sorunsuz bir şekilde base'e  39.920770, 32.854110  dönerseniz görev tamamlanır
 
