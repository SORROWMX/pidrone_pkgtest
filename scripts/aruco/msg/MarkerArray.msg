Header header
Marker[] markers
from pidrone_pkg.msg import Marker as ArucoMarker
from pidrone_pkg.msg import MarkerArray as ArucoMarkerArray
from dynamic_reconfigure.server import Server
try:
    from pidrone_pkg.cfg import MapConfig
except ImportError:
    # Если конфигурация не сгенерирована, создаем заглушку
    from dynamic_reconfigure.msg import Config as MapConfig