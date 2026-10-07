#CONSTANTS
#FPS
FPS=144
# How many physical updates will do in one render. Has big influence on performance 
# Higher values may cause FPS slowdown, but physics will be more stable especcialy on high speeds 
SUBSTEPS=3
PHYSICS_DT=1/120
#Display dimensions
WIDTH=1900
HEIGHT=1000
#Gravitational constant
GRAV_CONST=981
#Epsilon. Technical constant that has influence on special collision situations. 
EPS=10**(-9)

##Physics Worker config
MIN_UNRENDERED_BUFFER_SIZE=3000
MAX_BUFFER_SIZE=5000

##Joint Config
SECTOR_LENGTH = 50