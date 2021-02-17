# SmartFloorHeating
MQTT based PWM smart floor heating system, capable of regulating 3 rooms (more can easily be integrated), measure flow and return temperature. The current version passed a one year long test run and seems to run stable in any situations. However, the test mode will continue indefinied and code updates may be silently integrated. 
V1.1 is currently in development. Its key features are: 
- SQL Database integration
- Outdoor temperature forecast injection in order to handle temporary connection losses
- Publishing warnings if flow return temperature is too high
