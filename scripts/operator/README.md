VAZNE NAPOMENE!!!

Postupak je identican kao za instalaciju simulacije,preuzmete fajlove Dockerfile, entrypoint.sh i setup.sh. U setup.sh jedino mozete promjeniti ime kontejnera ili cjelokupan naziv slike ako zelite.

Kreirajte neki novi folder u koji cete ubaciti te fajlove.I uradite komandu "chmod +x setup.sh" da date odobrenje za izvrsavanje. Nakon toga jednostavno pokrenete setup.sh i kada on zavrsi pisace vam dalja uputstva kako da pokrenete kontejner i kada vam se pokrene bicete u folderu operator_ws. Sve je gotovo identicno kao za kontejner za simulaciju samo sto recimo umjesto komande run_husarion_nvidia kod operater se pominje komanda run_operator_nvidia,dakle samo su rijeci husarion i operator zammjenjene.

U root-u kontejnera na lokaciji /var/tmp se nalazi fajl cyclonedds.xml u kome se u tagu Peers unose svi korisnici Husarnet mreze sa kojima zelite da komunicirate,pise i u samom fajlu kako.

Takodje kada osposobite kontejner za simulaciju unutar foldera husarion_ws osim fajlova za simulaciju imacete folder scripts koji je istog sadrzaja kao ovaj u operator_ws. I isto na lokaciji /var/temp ce se nalaziti cyclonedds.xml unutar kontejnera za simulaciju. U operator_ws se nalazi folder scripts i u njemu fajlovi cyclonedds_script.sh i cyclonedds_local.sh, to su skripte koja postavlju promjenljive da bi se mogla vrsiti komunikacija.Postoje 2 slucaja:

I Komunikacija preko Husarnet-a odnosno dva odvojena racunara

U ovom slucaju cyclonedds.xml sadrzi dva korisnika simulation i operator.Znaci prilikom prijavljivanja na Husarnet bitno je da se jedan racunar poveze pod imenom "simulation", a drugi "operator".
Samo se uradi komanda "set_husarnet" da se postave parametri za Husarnet komunikaciju.

II Lokalna komunikacija na jednom laptopu,ako neko zeli lokalno da ima jedan kontejner kao operator a drugi za simulaciju

U ovom slucaju cyclonedds.xml nije bitan i samo se uradi "set_local" da se komunikacija izvrsava lokalno.

Ovi fajlovi ce se takodje nalaziti i u kontejneru za simulaciju,tako da ih i tamo morate source-ovati u zavisnosti od toga kako cete ih koristiti sa istim aliasima tj komandama.
U nastavku je komanda koju kopirate u vas ~/.bashrc fajl da mozete opet otvoriti operator kontejner

operator_connect() {
    xhost +local:root # Allow local Docker container to access X server
    local container_name="operator" # <--- CHANGE THIS if your container name is different
    local exec_target_name="operator" # <--- CHANGE THIS if exec target is different from start target (usually the same)

    # Check if the container is currently running
    if [ -n "$(docker ps -q -f name="^${container_name}$")" ]; then
        echo "Container '${exec_target_name}' is active. Attaching with exec..."
        docker exec -it "${exec_target_name}" /bin/bash
    else
        # Check if the container exists (even if stopped)
        if [ -n "$(docker ps -aq -f name="^${container_name}$")" ]; then
            echo "Container '${container_name}' exists but is not active. Starting..."
            if docker start "${container_name}"; then
                echo "Container '${container_name}' started successfully."
                echo "Attempting to attach now..."
                docker exec -it "${exec_target_name}" /bin/bash
            else
                echo "Failed to start container '${container_name}'. Check Docker logs."
            fi
        else
            echo "Container '${container_name}' does not exist. Please create or run it first."
            echo "Example: Use the 'docker run...' command provided in the setup instructions."
        fi
    fi
}

