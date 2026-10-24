# Middleware configuration

Reserve validated DDS/interface/discovery configuration here. No transport profile
has been validated. Use one installed RMW implementation and compatible bounded
QoS across hosts. Verify cross-container transport; host networking alone does not
establish discovery or shared-memory compatibility. Do not enable host IPC or
privileged mode as a default workaround. Domain IDs are not authorization.
