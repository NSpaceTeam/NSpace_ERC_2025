
# Husarnet Documentation Report

## 📌 Šta je Husarnet?

Husarnet je u suštini **peer-to-peer VPN** za komunikaciju niske latencije. Rešava problem komunikacije između uređaja kao što su ESP32 i Raspberry Pi u distribuiranim sistemima — posebno korisno u oblasti robotike.

Umesto klasičnog pristupa sa statičkim IP adresama, port-forwarding-om itd., Husarnet pravi **Layer 3 mesh mrežu (VPN)** u kojoj uređaji međusobno komuniciraju putem **privatnih IPv6 adresa**, kao da su u lokalnoj mreži (LAN).

> ❗ Napomena: Husarnet ne šalje nikakve podatke samostalno. On samo omogućava **vezu između uređaja**. Na nama je da definišemo koji protokol koristimo (npr. MQTT, ROS itd.).

Na Husarnet portalu, jedan uređaj može biti označen kao **ROS Master** — centralni čvor, kojem mreža daje prioritet.

---

## 🧪 Primer upotrebe 

- **Raspberry Pi** sa kamerom i motorima
- **Laptop** sa joystick-om i ekranom

> Napomena: Podrazumeva se da su oba uređaja dodata na Husarnet portal (opisano kasnije kako).

### ✅ Konfiguracija ROS Environment Variables:

**Raspberry (pretpostavljamo da je on ROS Master):**

```bash
# Na raspberry
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=robot
# Na laptopu:
export ROS_MASTER_URI=http://robot:11311
export ROS_HOSTNAME=laptop
```
## 💸 Pricing

> ⚠️ Na ovo treba obratiti pažnju jer u *Technical Handbook*-u piše da je **zabranjeno probijanje besplatnog plana**, osim ako oni to ne odobre.

- **Free plan:** Do 5 uređaja u jednoj mreži  
- **Plaćeni planovi:** Više od 5 uređaja  
- **Bez bandwith limita** – nema ograničenja u količini podataka koji se šalju

📌 **Napomena:** Na brzinu protoka podataka utiču:
- Brzina vaše internet konekcije
- Snaga CPU-a na uređajima

---


## 🛠️ Neophodno za rad (za rad jedne mreže)

1. Potrebno je napraviti nalog na njihovoj [platformi](https://husarnet.com/).
2. Napraviti svoj **network**.
3. Kada je napravljen network, dobija se pristupni **join code**.

🔑 **Moj join kod:**
```fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/hWhUasTYVtsMGg5jwViZQn```
---

## 🖥️ Konfiguracija na mašini koju želimo da dodamo na mrežu

### ✅ Instalacija Husarnet-a na Linux mašini:

```bash
curl https://install.husarnet.com/install.sh | sudo bash
# Dodavanje Linux-based mašine na mrežu:
husarnet join \
xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx/token \
imeUredjaja
```

### 🔌 Dodavanje ESP32-based uređaja na mrežu:
```
Husarnet.join(
    "xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx:xxxx/token",
    "mydevice");
Husarnet.start();
```

## 📡 **Uređaji koji se pojave na portalu su uspešno povezani.**

