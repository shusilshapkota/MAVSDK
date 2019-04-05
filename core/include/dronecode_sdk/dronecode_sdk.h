#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>

#include "deprecated.h"
#include "system.h"
#include "component_type.h"
#include "connection_result.h"


namespace dronecode_sdk {

class DronecodeSDKImpl;
class System;


/**
 * @brief This is the main class of Dronecode SDK (a MAVLink API Library for the Dronecode
 Platform).

 * It is used to discover vehicles and manage active connections.
 *
 * An instance of this class must be created (first) in order to use the library.
 * The instance must be destroyed after use in order to break connections and release all resources.
 */
class DronecodeSDK {
public:
    /** @brief Default UDP bind IP (accepts any incoming connections). */
    static constexpr auto DEFAULT_UDP_BIND_IP = "0.0.0.0";
    /** @brief Default UDP bind port (same port as used by MAVROS). */
    static constexpr int DEFAULT_UDP_PORT = 14540;
    /** @brief Default TCP remote IP (localhost). */
    static constexpr auto DEFAULT_TCP_REMOTE_IP = "127.0.0.1";
    /** @brief Default TCP remote port. */
    static constexpr int DEFAULT_TCP_REMOTE_PORT = 5760;
    /** @brief Default serial baudrate. */
    static constexpr int DEFAULT_SERIAL_BAUDRATE = 57600;

    /**
     * @brief Constructor.
     */
    DronecodeSDK();

    /**
     * @brief Destructor.
     *
     * Disconnects all connected vehicles and releases all resources.
     */
    ~DronecodeSDK();

    /**
     * @brief Adds Connection via URL
     *
     * Supports connection: Serial, TCP or UDP.
     * Connection URL format should be:
     * - UDP - udp://[Bind_host][:Bind_port]
     * - TCP - tcp://[Remote_host][:Remote_port]
     * - Serial - serial://Dev_Node[:Baudrate]
     *
     * @param connection_url connection URL string.
     * @return The result of adding the connection.
     */
    ConnectionResult add_any_connection(const std::string &connection_url);

    /**
     * @brief Adds a UDP connection to the specified port number.
     *
     * Any incoming connections are accepted (0.0.0.0).
     *
     * @param local_port The local UDP port to listen to (defaults to 14540, the same as MAVROS).
     * @return The result of adding the connection.
     */
    ConnectionResult add_udp_connection(int local_port = DEFAULT_UDP_PORT);

    /**
     * @brief Adds a UDP connection to the specified port number and local interface.
     *
     * To accept only local connections of the machine, use 127.0.0.1.
     * For any incoming connections, use 0.0.0.0.
     *
     * @param local_ip The local UDP IP address to listen to.
     * @param local_port The local UDP port to listen to (defaults to 14540, the same as MAVROS).
     * @return The result of adding the connection.
     */
    ConnectionResult add_udp_connection(const std::string &local_ip,
                                        int local_port = DEFAULT_UDP_PORT);

    /**
     * @brief Adds a TCP connection with a specific port number on localhost.
     *
     * @param remote_port The TCP port to connect to (defaults to 5760).
     * @return The result of adding the connection.
     */
    ConnectionResult add_tcp_connection(int remote_port = DEFAULT_TCP_REMOTE_PORT);

    /**
     * @brief Adds a TCP connection with a specific IP address and port number.
     *
     * @param remote_ip Remote IP address to connect to.
     * @param remote_port The TCP port to connect to (defaults to 5760).
     * @return The result of adding the connection.
     */
    ConnectionResult add_tcp_connection(const std::string &remote_ip,
                                        int remote_port = DEFAULT_TCP_REMOTE_PORT);

    /**
     * @brief Adds a serial connection with a specific port (COM or UART dev node) and baudrate as
     * specified.
     *
     *
     * @param dev_path COM or UART dev node name/path (e.g. "/dev/ttyS0", or "COM3" on Windows).
     * @param baudrate Baudrate of the serial port (defaults to 57600).
     * @return The result of adding the connection.
     */
    ConnectionResult add_serial_connection(const std::string &dev_path,
                                           int baudrate = DEFAULT_SERIAL_BAUDRATE);

    /**
     * @brief Possible configurations.
     */
    enum class Configuration {
        GroundStation, /**< @brief SDK is used as a ground station. */
        CompanionComputer /**< @brief SDK is used on a companion computer onboard the system (e.g.
                             drone). */
    };

    /**
     * @brief Set `Configuration` of SDK.
     *
     * The default configuration is `Configuration::GroundStation`
     * The configuration is used in order to set the MAVLink system ID, the
     * component ID, as well as the MAV_TYPE accordingly.
     *
     * @param configuration Configuration chosen.
     */
    void set_configuration(Configuration configuration);

    /**
     * @brief Type to call on discovery or timeout of a system.
     *
     * @param system Reference to timed out system.
     */
    using SystemCallback = std::function<void(System &system)>;

    /**
     * @brief Type to call on discovery or timeout of a component.
     *
     * @param system Reference to system.
     * @param component_type Component type.
     */
    using ComponentCallback = std::function<void(System &system, ComponentType component_type)>;

    /**
     * @brief Register callback for system discovery.
     *
     * @param callback Callback to register.
     */
    void register_on_system_discovery(SystemCallback callback);

    /**
     * @brief Register callback for system timeout.
     *
     * The callback will called when the last component times out.
     *
     * @param callback Callback to register.
     */
    void register_on_system_timeout(SystemCallback callback);

    /**
     * @brief Register callback for component discovery.
     *
     * The callback will called when any component is discovered.
     *
     * @param callback Callback to register.
     */
    void register_on_component_discovery(ComponentCallback callback);

    /**
     * @brief Register callback for component timeout.
     *
     * The callback will be called when any component times out.
     *
     * @param callback Callback to register.
     */
    void register_on_component_timeout(ComponentCallback callback);

    /**
     * @brief Get all systems ever discovered.
     *
     * If no system has been discovered yet, an empty vector is returned.
     *
     * @return A vector systems.
     */
    std::vector<std::shared_ptr<System>> systems() const;

    /**
     * @brief Get all systems currently connected.
     *
     * If no system is currently connected, an empty vector is returned.
     *
     * @return A vector of systems.
     */
    std::vector<std::shared_ptr<System>> systems_connected() const;

    /**
     * @brief Get all systems currently not connected.
     *
     * If all systems are currently connected, an empty vector is returned.
     *
     * @return A vector of systems.
     */
    std::vector<std::shared_ptr<System>> systems_disconnected() const;

    /**
     * @brief Get all systems ever discovered with a specific component.
     *
     * If no system with that component has been discovered yet, an empty vector is returned.
     *
     * @param component_type `ComponentType` to filter by.
     * @return A vector of systems.
     */
    std::vector<std::shared_ptr<System>> systems_with_component(ComponentType component_type) const;

    /**
     * @brief Get all systems currently connected with a specific component.
     *
     * If no system with that component is connected, an empty vector is returned.
     *
     * @param component_type `ComponentType` to filter by.
     * @return A vector of systems.
     */
    std::vector<std::shared_ptr<System>> systems_with_component_connected(ComponentType component_type) const;

    /**
     * @brief Get all systems currently disconnected with a specific component.
     *
     * If all systems with that component are connected, an empty vector is returned.
     *
     * @param component_type `ComponentType` to filter by.
     * @return A vector of systems.
     */
    std::vector<std::shared_ptr<System>> systems_with_component_disconnected(ComponentType component_type) const;

    /**
     * @brief Get the first discovered system (deprecated).
     *
     * This returns the first discovered system or a null system if no system has yet been found.
     *
     * This method is now deprecated. The new recommended way to get the first system is `systems()`.
     *
     * @return A reference to a system.
     */
    DEPRECATED System &system() const;

    /**
     * @brief Returns `true` if exactly one system is currently connected (deprecated).
     *
     * Connected means we are receiving heartbeats from this system.
     * It means the same as "discovered" and "not timed out".
     *
     * This method is deprecated, you can now check `System::is_connected()`.
     *
     * If multiple systems have connected, this will return `false`.
     *
     * @return `true` if exactly one system is connected.
     */
    DEPRECATED bool is_connected() const;

    /**
     * @brief Get vector of system UUIDs (deprecated).
     *
     * This returns a vector of the UUIDs of all systems that have been discovered.
     * If a system doesn't have a UUID then DronecodeSDK will instead use its MAVLink system ID
     * (range: 0..255).
     *
     * @note This method is deprecated because the UUID will be replaced
     *       by uid with 18 bytes.
     *
     * @return A vector containing the UUIDs.
     */
    DEPRECATED std::vector<uint64_t> system_uuids() const;

    /**
     * @brief Get the system with the specified UUID (deprecated).
     *
     * This returns a system for a given UUID if such a system has been discovered and a null
     * system otherwise.
     *
     * @note This method is deprecated because the UUID will be replaced
     *       by uid with 18 bytes.
     *
     * @param uuid UUID of system to get.
     * @return A reference to the specified system.
     */
    DEPRECATED System &system(uint64_t uuid) const;

    /**
     * @brief Callback type for discover and timeout notifications (deprecated).
     *
     * @note This typedef is deprecated because the UUID will be replaced
     *       by uid with 18 bytes.
     *
     * @param uuid UUID of system (or MAVLink system ID for systems that don't have a UUID).
     */
    typedef std::function<void(uint64_t uuid)> event_callback_t;

    /**
     * @brief Returns `true` if a system is currently connected (deprecated).
     *
     * Connected means we are receiving heartbeats from this system.
     * It means the same as "discovered" and "not timed out".
     *
     * @note This method is deprecated because the UUID will be replaced
     *       by uid with 18 bytes.
     *
     * @param uuid UUID of system to check.
     * @return `true` if system is connected.
     */
    DEPRECATED bool is_connected(uint64_t uuid) const;

    /**
     * @brief Register callback for system discovery (deprecated).
     *
     * This sets a callback that will be notified if a new system is discovered.
     *
     * If systems have been discovered before this callback is registered, they will be notified
     * at the time this callback is registered.
     *
     * This method is deprecated, please use `register_on_system_discovery`
     * or `register_on_component_discovery`.
     *
     * @note Only one callback can be registered at a time. If this function is called several
     * times, previous callbacks will be overwritten.
     *
     * @param callback Callback to register.
     *
     */
    DEPRECATED void register_on_discover(event_callback_t callback);


    /**
     * @brief Register callback for system timeout.
     *
     * This sets a callback that will be notified if no heartbeat of the system has been received
     * in 3 seconds.
     *
     * This method is deprecated, please use `register_on_system_timeout`
     * or `register_on_component_timeout`.
     *
     * @note Only one callback can be registered at a time. If this function is called several
     * times, previous callbacks will be overwritten.
     *
     * @param callback Callback to register.
     */
    DEPRECATED void register_on_timeout(event_callback_t callback);

private:
    /* @private. */
    std::unique_ptr<DronecodeSDKImpl> _impl;

    // Non-copyable
    DronecodeSDK(const DronecodeSDK &) = delete;
    const DronecodeSDK &operator=(const DronecodeSDK &) = delete;
};

} // namespace dronecode_sdk
