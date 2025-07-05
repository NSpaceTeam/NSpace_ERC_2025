VAZNE NAPOMENE!!!

Postupak je identican kao za instalaciju simulacije,preuzmete fajlove Dockerfile, entrypoint.sh i setup.sh.
U setup.sh jedino mozete promjeniti ime kontejnera ili cjelokupan naziv slike ako zelite.

Kreirajte neki novi folder u koji cete ubaciti te fajlove.I uradite komandu "chmod +x setup.sh" da date odobrenje za izvrsavanje.
Nakon toga jednostavno pokrenete setup.sh i kada on zavrsi pisace vam dalja uputstva kako da pokrenete kontejner i kada vam se pokrene bicete u folderu operator_ws.
U root-u kontejnera na lokaciji /var/tmp se nalazi fajl cyclonedds.xml 
u kome se u tagu Peers unose svi korisnici Husarnet mreze sa kojima zelite da komunicirate,pise i u samo fajlu kako.

Takodje kada osposobite kontejner za simulaciju unutar foldera husarion_ws osim fajlova za simulaciju imacete folder scripts koji je istog sadrzaja kao ovaj u operator_ws.
I isto na lokaciji /var/temp ce se nalaziti cyclonedds.xml unutar kontejnera za simulaciju.
U operator_ws se nalazi folder scripts i u njemu fajlovi cyclonedds_script.sh i cyclonedds_local.sh, to su skripte koja postavlju promjenljive da bi se mogla vrsiti komunikacija.Postoje 2 slucaja:

I Komunikacija preko Husarnet-a odnosno dva odvojena racunara
	U ovom slucaju u fajlu cyclonedds.xml se dodaju svi korisnici i samo se uradi source cyclonedds_script.sh


II Lokalna komunikacija na jednom laptopu,ako neko zeli lokalno da ima jedan kontejner kao operator a drugi za simulaciju
	U ovom slucaju cyclonedds.xml nije bitan i samo se uradi source cyclonedds_local.sh
	
Ovi fajlovi ce se takodje nalaziti i u kontejneru za simulaciju,tako da ih i tamo morate source-ovati u zavisnosti od toga kako cete ih koristiti.
