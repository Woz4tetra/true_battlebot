DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "bwbots"
    ;;

  dadbot-meatball)
    echo -n "bwbots"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
