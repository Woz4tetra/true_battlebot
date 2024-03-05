DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "~/.ssh/dadbot"
    ;;

  dadbot-meatball)
    echo -n "~/.ssh/dadbot"
    ;;

  dadbot-kgb)
    echo -n "~/.ssh/dadbot"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
