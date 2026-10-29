#!/bin/bash -e
# Copyright 2021 <Excelfore>
# Provides: install_esync.sh
# Author: Excelfore
# Short-Description: This script is used to install debian SDK and setup the eSync-client.

red=196; green=40; yellow=190; magenta=93; cyan=87; normal=0

clc() {
    printf "\033[38;5;${1}m"
}

printc() {
    local color=$1; shift
    local ctrl=$(clc $color)
    local res='\033[0m'
    local fmt=$1; shift
    [ $color = 0 ] && ctrl=$res
    printf "${ctrl}${fmt}${res}\n" "$@"
}

THIS_DIR="$(realpath $(dirname $0))"

platform_confFile=${THIS_DIR}/sdk.cfg
if [ ! -f $platform_confFile ]; then
    printc $red "ERROR: sdk.cfg file not found in $THIS_DIR"
    exit 1
fi

. $platform_confFile

sdk_confFile=$ESYNC_INSTALL_DIR/esync/config.cfg
ESYNC_DIR=$ESYNC_INSTALL_DIR/esync

usage() {
    cat <<USAGE
Usage options :

$0 <... options ...>

Install options :
    -i	# Install and start eSync.
    -n	# Install eSync without starting it.
    -s	# Start eSync.
    -p	# Stop eSync.
    -S  # Report eSync status.
    -u  # Stop and uninstall eSync.
    -U  # -u plus remove provisioned data.
    -c  # Start CAN ECU Simulator (use with -i or -s).
    -w  # Docker mode: with -i creates Ubuntu 16.04 container, with -u stops/removes containers.
    -Q  # Test mode: start eSync with esyncboot_test.json (use with -i or -u).
    -y  # Use install defaults, don't ask for confirmation.
    -v  # Verbose mode - show detailed esync commands.
    -vv  # Debug mode - enable bash trace (set -ex).

Other options :
    -h           # Help / Usage Options

e.g., './install_esync.sh -i'
e.g., './install_esync.sh -i -Q' (test mode install)
e.g., './install_esync.sh -u -Q' (test mode uninstall)
e.g., './install_esync.sh -i -v' (verbose)
e.g., './install_esync.sh -i -vv' (debug trace)

eSync logs can be found in ${HOME}/esync/data/log/

USAGE
    exit 1
}

# Check if no arguments were provided
if [ $# -eq 0 ]; then
    usage
fi

userRead() {
    if $usedefault; then
        echo $2
    else
        read -p "$1" userInput
        echo ${userInput}
    fi
}

retryCount=0
failCount=0
rpiIPFailCount=0
rpiPassFailCount=0
rpiNameFailCount=0
secretFailCount=0
apikeyFailCount=0
board1Count=0
dmtreeProvisionCount=0
typeHandlerCount=0
dmtreeInputCount=0
readFirstBoardCount=0
HardwareInstallDirCount=0
emptyInputCount0=0
emptyInputCount1=0

total_hash=50
show_progress()
{
    tasks_in_total=$1
    space=$(($total_hash+3-tasks_in_total))
    c=""
    s=""
    for current_task in $(seq $tasks_in_total) 
    do
    c=`echo $c▇`
    done

    frmt=`echo "%${space}d"`
    printf "$c $frmt\r" "$2"
    if [ $tasks_in_total -eq $total_hash ];then
        printf "\n"
    fi
}

verify_platform_config()
{
    # Parse JSON file for provision-key.json
    provision_key_file="${THIS_DIR}/provision-key.json"
    if [ -f "$provision_key_file" ]; then
        APIKEY=$(grep -o '"apikey"[[:space:]]*:[[:space:]]*"[^"]*"' "$provision_key_file" | cut -d'"' -f4)
        SECRET=$(grep -o '"secret"[[:space:]]*:[[:space:]]*"[^"]*"' "$provision_key_file" | cut -d'"' -f4)
        DOMAIN=$(grep -o '"domain"[[:space:]]*:[[:space:]]*"[^"]*"' "$provision_key_file" | cut -d'"' -f4)
    else
        printc $magenta "***********************************************************************"
        printc $magenta "  Error: provision-key.json file not found in $THIS_DIR"
        printc $magenta "***********************************************************************"
        exit 1
    fi
    
    if [ -z "$ESYNC_INSTALL_DIR" ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "  Error: \"ESYNC_INSTALL_DIR\" missing in $platform_confFile"
        printc $magenta "***********************************************************************"
        exit 1
    fi
    if [ -z "$APIKEY" ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "  Error: \"apikey\" missing in provision-key.json"
        printc $magenta "***********************************************************************"
        exit 1
    fi
    if [ -z "$SECRET" ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "  Error: \"secret\" missing in provision-key.json"
        printc $magenta "***********************************************************************"
        exit 1
    fi
}

esync_copy() {
    if  [ ! -d "$ESYNC_DIR" ] ; then
        mkdir -p $ESYNC_DIR
        INSTALL_ROOT_DIR=$(dirname $ESYNC_DIR)
        tar -xvf $THIS_DIR/esync.tar.gz -C $INSTALL_ROOT_DIR >> /dev/null
        if [ -f $ESYNC_DIR/preinst.sh ];then
            $exe $ESYNC_DIR/preinst.sh
        fi
        cp $THIS_DIR/sdk.cfg $ESYNC_DIR
        cp $THIS_DIR/provision-key.json $ESYNC_DIR
        cp $THIS_DIR/esync_version $ESYNC_DIR
        platform_confFile=$ESYNC_DIR/sdk.cfg
        . $sdk_confFile

        mkdir -p $ESYNC_ETC
        . $ESYNC_DIR/sdk.cfg
        mkdir -p $ESYNC_LOG
        mkdir -p $ESYNC_STATE

        MTYPE=`uname -m`
        if [[ $MTYPE = 'x86_64' && -f $ESYNC_DIR/bin/esync-bridge ]]; then
            mkdir -p $ESYNC_RUN_DIR/etc/bridgecfg/tmp
            servercfg=server-config
            cp $ESYNC_DIR/etc/config/bridgecfg/$servercfg.zip $ESYNC_RUN_DIR/etc/bridgecfg/tmp
            cp $ESYNC_DIR/etc/config/bridgecfg/bridge-config.zip $ESYNC_RUN_DIR/etc/bridgecfg
            cd $ESYNC_RUN_DIR/etc/bridgecfg/tmp
            unzip $servercfg.zip >> /dev/null
            sed -i 's:/etc/ssl/certs/ExcelforeDev-CA.pem:'"${ESYNC_RUN_DIR}"/etc/esync/provision/edatx-ca.pem':g' servers4.json
            sed -i 's:/etc/ssl/certs/:'"${ESYNC_RUN_DIR}"/etc/esync/provision':g' servers4.json
            zip -r $servercfg.zip servers4.json >> /dev/null
            cp $servercfg.zip $ESYNC_RUN_DIR/etc/bridgecfg
            cd ..
            rm -rf $ESYNC_RUN_DIR/etc/bridgecfg/tmp
        else
            if [ -d $ESYNC_DIR/etc/config/bridgecfg ];then
                mkdir -p $ESYNC_RUN_DIR/etc/bridgecfg
                cp $ESYNC_DIR/etc/config/bridgecfg/* $ESYNC_RUN_DIR/etc/bridgecfg/
            fi
        fi
        if [ -f $ESYNC_DIR/postinst.sh ];then
            $exe $ESYNC_DIR/postinst.sh
        fi

        if [ -f "$THIS_DIR/.docker_image_track" ]; then
            mv "$THIS_DIR/.docker_image_track" "$ESYNC_DIR/.docker_image_track"
        fi
    else
        printc $magenta "***********************************************************************"
        printc $magenta "esync exists at $ESYNC_DIR. You must uninstall first"
        printc $magenta "***********************************************************************"
        exit 1
    fi
}

esync_start() {
    if  [ -f "$ESYNC_DIR/start.sh" ] ; then
        start_args="-s -o $ESYNC_DIR -d $ESYNC_DATA -w"
        if $can_simulator; then
            start_args="$start_args -c"
        fi
        if $testmode; then
            start_args="$start_args -Q"
        fi
        $exe $ESYNC_DIR/start.sh $start_args
    else
        printc $magenta "***********************************************************************"
        printc $magenta "esync not installed at $ESYNC_DIR"
        printc $magenta "***********************************************************************"
    fi
}

esync_stop() {
    if  [ -f "$ESYNC_DIR/start.sh" ] ; then
        stop_args="-k -o $ESYNC_DIR -d $ESYNC_DATA"
        if $testmode; then
            stop_args="$stop_args -Q"
        fi
        $exe $ESYNC_DIR/start.sh $stop_args
        printc $magenta "*** stopped ***"
    else
        printc $magenta "***********************************************************************"
        printc $magenta "esync not installed at $ESYNC_DIR"
        printc $magenta "***********************************************************************"
    fi
}

esync_remove() {
    if [ $removeall ];then
        rm -rf $ESYNC_CERTS && rm -rf $ESYNC_DATA && rm -rf $ESYNC_ETC && rm -rf $ESYNC_LOG
        if [ -f $ESYNC_DIR/postrm.sh ];then
            $exe $ESYNC_DIR/postrm.sh
        fi
    fi
    rm -rf $ESYNC_DIR/../esync.tar.gz $ESYNC_DIR
    printc $cyan "***********************************************************************"
    printc $cyan "eSync Uninstalled"
    printc $cyan "***********************************************************************"
}

esync_status()
{
    quiet=$1

    if [ ! -f $ESYNCBOOT_BIN ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "eSync not installed $ESYNCBOOT_BIN"
        printc $magenta "***********************************************************************"
        exit 1
    fi

    if [ ! -f $ESYNC_DIR/start.sh ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "eSync not installed $ESYNC_DIR/start.sh"
        printc $magenta "***********************************************************************"
        exit 1
    fi

    if $exe $ESYNC_DIR/start.sh $quiet -o $ESYNC_DIR -d $ESYNC_DATA; then
        printc $green "***********************************************************************"
        printc $green "eSync Installed and running"
        printc $green "***********************************************************************"
    else
        printc $cyan "***********************************************************************"
        printc $cyan "eSync not running !"
        printc $cyan "***********************************************************************"
    fi
}

esync_wait()
{
    if [ ! -f $ESYNC_DIR/start.sh ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "eSync not installed $ESYNC_DIR/start.sh"
        printc $magenta "***********************************************************************"
        exit 1
    fi

    wait_args="-w $verbose_flag -o $ESYNC_DIR -d $ESYNC_DATA"
    if $can_simulator; then
        wait_args="$wait_args -c"
    fi
    if $testmode; then
        wait_args="$wait_args -Q"
    fi
    $exe $ESYNC_DIR/start.sh $wait_args
}

docker_check_and_pull()
{
    printc $cyan "***********************************************************************"
    printc $cyan "Docker Mode: Setting up Ubuntu container for update demo"
    printc $cyan "***********************************************************************"

    # Check if Docker is installed
    if ! command -v docker > /dev/null 2>&1; then
        printc $red "***********************************************************************"
        printc $red "Error: Docker is not installed on this system"
        printc $red "***********************************************************************"
        exit 1
    fi

    # Ask user for Ubuntu version
    printc $yellow "***********************************************************************"
    printc $yellow "Select Ubuntu version:"
    printc $yellow "  1) 16.04"
    printc $yellow "  2) 18.04"
    printc $yellow "  3) 20.04"
    printc $yellow "  4) 22.04"
    printc $yellow "  5) 24.04"
    printc $yellow "***********************************************************************"
    printc $cyan "Enter choice (1-5) or press Enter for default (16.04):"
    read -p "> " ubuntu_choice

    # Set ubuntu version based on choice
    case "$ubuntu_choice" in
        1|"")
            ubuntu_version="16.04"
            ;;
        2)
            ubuntu_version="18.04"
            ;;
        3)
            ubuntu_version="20.04"
            ;;
        4)
            ubuntu_version="22.04"
            ;;
        5)
            ubuntu_version="24.04"
            ;;
        *)
            printc $red "***********************************************************************"
            printc $red "Invalid choice. Using default: 16.04"
            printc $red "***********************************************************************"
            ubuntu_version="16.04"
            ;;
    esac

    printc $green "Selected Ubuntu version: $ubuntu_version"

    # Ask user for container name (press Enter for default)
    printc $yellow "Enter container name (press Enter for default: xl4-test-container):"
    read -p "> " container_name

    # Use default if empty
    if [ -z "$container_name" ]; then
        container_name="xl4-test-container"
        printc $cyan "Using default container name: $container_name"
    fi

    printc $yellow "***********************************************************************"
    printc $yellow "Container name: $container_name"
    printc $yellow "***********************************************************************"

    # Check if container with this name already exists
    existing_container=$(docker ps -a --filter "name=^${container_name}$" --format "{{.Names}}")

    if [ -n "$existing_container" ]; then
        printc $yellow "***********************************************************************"
        printc $yellow "Container '$container_name' already exists!"
        printc $yellow "***********************************************************************"
        printc $yellow "Container Details:"
        printc $yellow "***********************************************************************"

        # Display container in table format like docker ps -a
        docker ps -a --filter "name=^${container_name}$"

        printc $yellow "***********************************************************************"

        # Ask what to do
        printc $cyan "What would you like to do?"
        printc $cyan "  1) Use existing container (no changes)"
        printc $cyan "  2) Remove and recreate with ubuntu:16.04"
        printc $cyan "  3) Enter a different container name"
        read -p "Enter choice (1/2/3): " choice

        case "$choice" in
            1)
                printc $green "***********************************************************************"
                printc $green "Using existing container '$container_name'"
                printc $green "***********************************************************************"
                docker ps -a --filter "name=^${container_name}$"
                return 0
                ;;
            2)
                printc $magenta "***********************************************************************"
                printc $magenta "Removing existing container '$container_name'..."
                printc $magenta "***********************************************************************"

                # Stop if running
                is_running=$(docker ps -q --filter "name=^${container_name}$")
                if [ -n "$is_running" ]; then
                    printc $magenta "Stopping container..."
                    if ! docker stop $container_name; then
                        printc $red "***********************************************************************"
                        printc $red "Error: Failed to stop container"
                        printc $red "***********************************************************************"
                        exit 1
                    fi
                fi

                # Remove container
                printc $magenta "Removing container..."
                if ! docker rm $container_name; then
                    printc $red "***********************************************************************"
                    printc $red "Error: Failed to remove container"
                    printc $red "***********************************************************************"
                    exit 1
                fi
                printc $green "Container removed successfully"
                ;;
            3)
                printc $yellow "Enter new container name:"
                read -p "> " new_name
                if [ -z "$new_name" ]; then
                    printc $red "***********************************************************************"
                    printc $red "Error: Container name cannot be empty"
                    printc $red "***********************************************************************"
                    exit 1
                fi
                container_name="$new_name"
                printc $cyan "Using new container name: $container_name"

                # Re-check if the new container name already exists
                existing_container=$(docker ps -a --filter "name=^${container_name}$" --format "{{.Names}}")
                if [ -n "$existing_container" ]; then
                    printc $red "***********************************************************************"
                    printc $red "Error: Container '$container_name' already exists"
                    printc $red "***********************************************************************"
                    exit 1
                fi
                # Continue to image pull/creation below
                ;;
            *)
                printc $red "***********************************************************************"
                printc $red "Invalid choice. Exiting."
                printc $red "***********************************************************************"
                exit 1
                ;;
        esac
    fi

    # Check if image already exists locally
    printc $yellow "***********************************************************************"
    printc $yellow "Checking for ubuntu:$ubuntu_version image..."
    printc $yellow "***********************************************************************"

    image_pulled=false
    if docker image inspect ubuntu:$ubuntu_version > /dev/null 2>&1; then
        printc $green "***********************************************************************"
        printc $green "Image ubuntu:$ubuntu_version already exists locally - SKIP"
        printc $green "***********************************************************************"
    else
        printc $yellow "***********************************************************************"
        printc $yellow "Pulling ubuntu:$ubuntu_version image from Docker Hub..."
        printc $yellow "***********************************************************************"

        if docker pull ubuntu:$ubuntu_version; then
            printc $green "Successfully pulled ubuntu:$ubuntu_version image"
            image_pulled=true
        else
            printc $red "***********************************************************************"
            printc $red "Error: Failed to pull ubuntu:$ubuntu_version image"
            printc $red "***********************************************************************"
            exit 1
        fi
    fi

    # Create and start the container
    printc $yellow "***********************************************************************"
    printc $yellow "Creating and starting container '$container_name' with ubuntu:$ubuntu_version..."
    printc $yellow "***********************************************************************"

    if docker run -d --name "$container_name" ubuntu:$ubuntu_version /bin/bash -c "echo 'Ubuntu $ubuntu_version container running'; sleep infinity"; then
        printc $green "***********************************************************************"
        printc $green "Container '$container_name' created and started successfully!"
        printc $green "***********************************************************************"
        docker ps --filter "name=^${container_name}$"

        # Create tracking file only if we pulled the image
        if [ "$image_pulled" = "true" ]; then
            DOCKER_TRACK_FLAG="$THIS_DIR/.docker_image_track"
            echo "DOCKER_IMAGE=ubuntu:$ubuntu_version" > "$DOCKER_TRACK_FLAG"
        fi
    else
        printc $red "***********************************************************************"
        printc $red "Error: Failed to create container"
        printc $red "***********************************************************************"
        exit 1
    fi
}

docker_stop_all()
{
    printc $cyan "***********************************************************************"
    printc $cyan "Checking for Docker containers to stop and remove"
    printc $cyan "***********************************************************************"

    # Check if Docker is installed
    if ! command -v docker > /dev/null 2>&1; then
        printc $red "***********************************************************************"
        printc $red "Error: Docker is not installed on this system"
        printc $red "***********************************************************************"
        exit 1
    fi

    # Get list of all containers (running and stopped) using docker ps -a
    all_containers=$(docker ps -a -q)

    if [ -z "$all_containers" ]; then
        printc $yellow "***********************************************************************"
        printc $yellow "No Docker containers found"
        printc $yellow "***********************************************************************"
    else
        removed_count=0

        # Loop through each container and ask for confirmation
        for container_id in $all_containers; do
            printc $yellow "***********************************************************************"
            printc $yellow "Container Details:"
            printc $yellow "***********************************************************************"

            # Display container in table format like docker ps -a
            docker ps -a --filter "id=$container_id"

            printc $yellow "***********************************************************************"

            # Ask for confirmation for this container
            user_response=$(userRead "Stop and remove this container? (yes/no): " "no")

            if [ "$user_response" = "yes" ] || [ "$user_response" = "y" ] || [ "$user_response" = "YES" ] || [ "$user_response" = "Y" ]; then
                # Check if container is running and stop it
                is_running=$(docker ps -q --filter "id=$container_id")
                if [ -n "$is_running" ]; then
                    printc $magenta "Stopping container $container_id..."
                    if ! docker stop $container_id; then
                        printc $red "***********************************************************************"
                        printc $red "Error: Failed to stop container $container_id"
                        printc $red "***********************************************************************"
                        continue
                    fi
                fi

                # Remove the container
                printc $magenta "Removing container $container_id..."
                if docker rm $container_id; then
                    printc $green "Successfully removed container $container_id"
                    removed_count=$((removed_count + 1))
                else
                    printc $red "***********************************************************************"
                    printc $red "Error: Failed to remove container $container_id"
                    printc $red "***********************************************************************"
                fi
            else
                printc $cyan "Skipping container $container_id"
            fi
        done

        if [ $removed_count -gt 0 ]; then
            printc $green "***********************************************************************"
            printc $green "Successfully processed $removed_count container(s)"
            printc $green "***********************************************************************"
        else
            printc $cyan "***********************************************************************"
            printc $cyan "No containers were removed"
            printc $cyan "***********************************************************************"
        fi
    fi

    # Check if docker-image.cfg exists and remove the image
    DOCKER_CONFIG_FILE="$ESYNC_DATA/workload-agent/docker-image.cfg"
    if [ -f "$DOCKER_CONFIG_FILE" ]; then
        . "$DOCKER_CONFIG_FILE"

        printc $magenta "***********************************************************************"
        printc $magenta "Removing Docker image $DOCKER_IMAGE..."
        printc $magenta "***********************************************************************"
        if docker rmi $DOCKER_IMAGE; then
            printc $green "***********************************************************************"
            printc $green "Successfully removed image $DOCKER_IMAGE"
            printc $green "***********************************************************************"
        else
            printc $red "***********************************************************************"
            printc $red "Error: Failed to remove image $DOCKER_IMAGE"
            printc $red "***********************************************************************"
        fi

        # Remove the tracking files
        rm -f "$DOCKER_CONFIG_FILE"
        rm -f "$ESYNC_DIR/.docker_image_track"
        rm -f "$THIS_DIR/.docker_image_track"
    fi
}

esync_verify() {
    if [ ! -f $HM_BIN ]; then
        printc $magenta "***********************************************************************"
        printc $magenta "eSync not installed $HM_BIN"
        printc $magenta "***********************************************************************"
        exit 1
    fi

    printc $magenta "***********************************************************************"
    printc $magenta "Getting Server Certificate & URL !!!"
    printc $magenta "***********************************************************************"

    for count in $(seq 1 10)
    do
        if [ -f  $CERT_CONF ] && [ -f $URL_CONF ];then
            for count in $(seq $count 10)
            do
                show_progress $((count*5)) $((count*10))
                sleep 0.01
            done
            printc $magenta "***********************************************************************"
            printc $magenta "Certificate and URL received!!, Provisioning eSync Tree Now....!!!"
            printc $magenta "***********************************************************************"
            break
        elif [ $count -ge 10 ];then
            printc $red "*************************************************************************************************"
            printc $red "Certificate and URL not found, marking Installation failed and cleaning up installation !!! "
            printc $red "*************************************************************************************************"
            esync_stop
            # esync_remove
            exit 1
        else
            show_progress $((count*5)) $((count*10))
            sleep 0.5
        fi
        count=$count+1;
        sleep 1
    done

    for count in $(seq 1 50)
    do
        if [ -f $EOL_TRIGGERED ];then
            for count in $(seq $count 50)
            do
                show_progress ${count} $(($count*2))
                sleep 0.01
            done
            printc $magenta "****************************************"
            printc $magenta "eSync Tree provisioned Successfully!!"
            printc $magenta "****************************************"
            break
        elif [ $count -ge 50 ];then
            printc $red "***********************************************************************************************"
            printc $red "eSync Tree not provisioned Failed !! Contact Excelfore Corp for the support!!!"
            printc $red "***********************************************************************************************"
            exit 1
        else
            show_progress ${count} $(($count*2))
            sleep 1
        fi
    done
    esync_status -q
}

install=false
start=false
stop=false
uninstall=false
removeall=false
status=false
usedefault=false
verbose_level=0
can_simulator=false
dockermode=false
testmode=false
exe="bash"
while getopts "inspSuUcyvwQh" opt; do
    case "$opt" in
        "i")
            install=true
            start=true
            ;;
        "n") install=true;;
        "s") start=true;;
        "p") stop=true;;
        "S") status=true;;
        "u") uninstall=true;;
        "U")
             uninstall=true
             removeall=true
             ;;
        "c") can_simulator=true;;
        "y") usedefault=true;;
        "v") verbose_level=$((verbose_level + 1));;
        "w") dockermode=true;;
        "Q") testmode=true;;
        "h") printc $green " Help Options :"
             usage;;
          *) printc $red " [error]: Invalid option"
             usage;;
    esac
done
shift $((OPTIND-1))

# Handle verbose levels
if [ $verbose_level -eq 1 ]; then
    # -v: pass verbose flag to start.sh (shows detailed commands)
    verbose_flag="-v"
elif [ $verbose_level -ge 2 ]; then
    # -vv: enable bash trace for debugging
    set -ex
    exe="bash -ex"
    verbose_flag="-v"
else
    verbose_flag=""
fi

if $install; then
    if $dockermode; then
        docker_check_and_pull
    fi
    verify_platform_config
    esync_copy
fi

[ -f $sdk_confFile ] && . $sdk_confFile
export ESYNC_INSTALL_DIR
export ESYNC_DIR=$ESYNC_INSTALL_DIR/esync

if $uninstall; then
    if $dockermode; then
        docker_stop_all
    fi
    esync_stop
    esync_remove
elif $start; then
    esync_start
    esync_wait
    #esync_verify
elif $stop; then
    esync_stop
elif $status; then
    esync_status
fi
exit 0




