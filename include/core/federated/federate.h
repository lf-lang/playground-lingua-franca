/**
 * @file
 * @author Soroush Bateni
 * @author Peter Donovan
 * @author Edward A. Lee
 * @author Anirudh Rengarajsm
 * @copyright (c) 2020-2023, The University of California at Berkeley.
 * License: <a href="https://github.com/lf-lang/reactor-c/blob/main/LICENSE.md">BSD 2-clause</a>
 * @brief Data structures and functions used and defined in federate.c.
 */

#ifndef FEDERATE_H
#define FEDERATE_H

#include <stdbool.h>

#include "tag.h"
#include "lf_types.h"
#include "environment.h"
#include "low_level_platform.h"

#ifndef ADVANCE_MESSAGE_INTERVAL
#define ADVANCE_MESSAGE_INTERVAL MSEC(10)
#endif

//////////////////////////////////////////////////////////////////////////////////
// Data types

/**
 * Structure that a federate instance uses to keep track of its own state.
 */
typedef struct federate_instance_t {
  /**
   * The TCP socket descriptor for this federate to communicate with the RTI.
   * This is set by lf_connect_to_rti(), which must be called before other
   * functions that communicate with the rti are called.
   */
  int socket_TCP_RTI;

  /**
   * Thread listening for incoming TCP messages from the RTI.
   */
  lf_thread_t RTI_socket_listener;

  /**
   * Number of inbound physical connections to the federate.
   * This can be either physical connections, or logical connections
   * in the decentralized coordination, or both.
   */
  size_t number_of_inbound_p2p_connections;

  /**
   * Array of thread IDs for threads that listen for incoming messages.
   * This is NULL if there are none and otherwise has size given by
   * number_of_inbound_p2p_connections.
   */
  lf_thread_t* inbound_socket_listeners;

  /**
   * Number of outbound peer-to-peer connections from the federate.
   * This can be either physical connections, or logical connections
   * in the decentralized coordination, or both.
   */
  size_t number_of_outbound_p2p_connections;

  /**
   * An array that holds the socket descriptors for inbound
   * connections from each federate. The index will be the federate
   * ID of the remote sending federate. This is initialized at startup
   * to -1 and is set to a socket ID by lf_handle_p2p_connections_from_federates()
   * when the socket is opened.
   *
   * @note There will not be an inbound socket unless a physical connection
   * or a p2p logical connection (by setting the coordination target property
   * to "distributed") is specified in the Lingua Franca program where this
   * federate is the destination. Multiple incoming p2p connections from the
   * same remote federate will use the same socket.
   */
  int sockets_for_inbound_p2p_connections[NUMBER_OF_FEDERATES];

  /**
   * An array that holds the socket descriptors for outbound direct
   * connections to each remote federate. The index will be the federate
   * ID of the remote receiving federate. This is initialized at startup
   * to -1 and is set to a socket ID by lf_connect_to_federate()
   * when the socket is opened.
   *
   * @note This federate will not open an outbound socket unless a physical
   * connection or a p2p logical connection (by setting the coordination target
   * property to "distributed") is specified in the Lingua Franca
   * program where this federate acts as the source. Multiple outgoing p2p
   * connections to the same remote federate will use the same socket.
   */
  int sockets_for_outbound_p2p_connections[NUMBER_OF_FEDERATES];

  /**
   * Thread ID for a thread that accepts sockets and then supervises
   * listening to those sockets for incoming P2P (physical) connections.
   */
  lf_thread_t inbound_p2p_handling_thread_id;

  /**
   * A socket descriptor for the socket server of the federate.
   * This is assigned in lf_create_server().
   * This socket is used to listen to incoming physical connections from
   * remote federates. Once an incoming connection is accepted, the
   * opened socket will be stored in
   * federate_sockets_for_inbound_p2p_connections.
   */
  int server_socket;

  /**
   * The port used for the server socket to listen for messages from other federates.
   * The federate informs the RTI of this port once it has created its socket server by
   * sending an ADDRESS_AD message (@see rti.h).
   */
  int server_port;

  /**
   * Most recent tag advance grant (TAG) received from the RTI, or NEVER if none
   * has been received. This variable should only be accessed while holding the
   * mutex lock on the top-level environment.
   */
  tag_t last_TAG;

  /**
   * Indicates whether the last TAG received is provisional or an ordinary TAG.
   * If the last TAG has been provisional, network port absent reactions must be inserted.
   * This variable should only be accessed while holding the mutex lock.
   */
  bool is_last_TAG_provisional;

  /**
   * Indicator of whether this federate has upstream federates.
   * The default value of false may be overridden in _lf_initialize_trigger_objects.
   */
  bool has_upstream;

  /**
   * Indicator of whether this federate has downstream federates.
   * The default value of false may be overridden in _lf_initialize_trigger_objects.
   */
  bool has_downstream;

  /**
   * Used to prevent the federate from sending a REQUEST_STOP
   * message if it has already received a stop request from the RTI.
   * This variable should only be accessed while holding a mutex lock.
   */
  bool received_stop_request_from_rti;

  /**
   * A record of the most recently sent LTC (latest tag confirmed) message.
   * In some situations, federates can send logical_tag_complete for
   * the same tag twice or more in-a-row to the RTI. For example, when
   * _lf_next() returns without advancing tag. To prevent overwhelming
   * the RTI with extra messages, record the last sent logical tag
   * complete message and check against it in lf_latest_tag_confirmed().
   *
   * @note Here, the underlying assumption is that the TCP stack will
   *  deliver the Logical TAG Complete message to the RTI eventually
   *  if it is deliverable
   */
  tag_t last_sent_LTC;

  /**
   * A record of the most recently sent NET (next event tag) signal.
   */
  tag_t last_sent_NET;

  /**
   * A record of the most recently skipped NET (next event tag) signal.
   */
  tag_t last_skipped_NET;

  /**
   * Indicator of whether this federate has received any DNET (downstream next event tag) signal.
   */
  bool received_any_DNET;

  /**
   * A record of the most recent DNET (downstream next event tag) signal.
   */
  tag_t last_DNET;

  /**
   * For use in federates with centralized coordination, the minimum
   * time delay between a physical action within this federate and an
   * output from this federate.  This is NEVER if there is causal
   * path from a physical action to any output.
   */
  instant_t min_delay_from_physical_action_to_federate_output;

#ifdef FEDERATED_DECENTRALIZED
  /**
   * Thread responsible for setting ports to absent by an STAA offset if they
   * aren't already known.
   */
  lf_thread_t staaSetter;
#endif
} federate_instance_t;

#ifdef FEDERATED_DECENTRALIZED
typedef struct staa_t {
  lf_action_base_t** actions;
  size_t STAA;
  size_t num_actions;
} staa_t;
#endif

typedef struct federation_metadata_t {
  const char* federation_id;
  char* rti_host;
  int rti_port;
  char* rti_user;
} federation_metadata_t;

typedef enum parse_rti_code_t { SUCCESS, INVALID_PORT, INVALID_HOST, INVALID_USER, FAILED_TO_PARSE } parse_rti_code_t;

//////////////////////////////////////////////////////////////////////////////////
// Global variables

/**
 * Mutex lock held while performing outbound socket write and close operations.
 */
extern lf_mutex_t lf_outbound_socket_mutex;

/**
 * Condition variable for blocking on unkonwn federate input ports.
 */
extern lf_cond_t lf_port_status_changed;

//////////////////////////////////////////////////////////////////////////////////
// Public functions (in alphabetical order)

/**
 * @brief Connect to the federate with the specified id.
 *
 * The established connection will then be used in functions such as lf_send_tagged_message()
 * to send messages directly to the specified federate.
 * This function first sends an MSG_TYPE_ADDRESS_QUERY message to the RTI to obtain
 * the IP address and port number of the specified federate. It then attempts
 * to establish a socket connection to the specified federate.
 * If this fails, the program exits. If it succeeds, it sets element [id] of
 * the _fed.sockets_for_outbound_p2p_connections global array to
 * refer to the socket for communicating directly with the federate.
 * @param remote_federate_id The ID of the remote federate.
 */
void lf_connect_to_federate(uint16_t);

/**
 * @brief Connect to the RTI at the specified host and port.
 *
 * This will return the socket descriptor for the connection.
 * If port_number is 0, then start at DEFAULT_PORT and increment
 * the port number on each attempt. If an attempt fails, wait CONNECT_RETRY_INTERVAL
 * and try again.  If it fails after CONNECT_TIMEOUT, the program exits.
 * If it succeeds, it sets the _fed.socket_TCP_RTI global variable to refer to
 * the socket for communicating with the RTI.
 * @param hostname A hostname, such as "localhost".
 * @param port_number A port number or 0 to start with the default.
 */
void lf_connect_to_rti(const char* hostname, int port_number);

/**
 * @brief Create a server to listen to incoming P2P connections.
 *
 * Such connections are used for physical connections or any connection if using
 * decentralized coordination. This function only handles the creation of the server socket.
 * The bound port for the server socket is then sent to the RTI by sending an
 * MSG_TYPE_ADDRESS_ADVERTISEMENT message (@see net_common.h).
 * This function expects no response from the RTI.
 *
 * If a port is specified by the user, that will be used.
 * Otherwise, a random port will be assigned.  If the bind fails,
 * it will retry after PORT_BIND_RETRY_INTERVAL until it has tried
 * PORT_BIND_RETRY_LIMIT times. Then it will fail.
 *
 * @param specified_port The port specified by the user or 0 to use a random port.
 */
void lf_create_server(int specified_port);

/**
 * @brief Enqueue port absent reactions.
 *
 * These reactions will send a MSG_TYPE_PORT_ABSENT
 * message to downstream federates if a given network output port is not present.
 * @param env The environment of the federate
 */
void lf_enqueue_port_absent_reactions(environment_t* env);

/**
 * @brief Thread to accept connections from other federates.
 *
 * This thread accepts connections from federates that send messages directly
 * to this one (not through the RTI). This thread starts a thread for
 * each accepted socket connection to read messages and, once it has opened all expected
 * sockets, exits.
 * @param ignored No argument needed for this thread.
 */
void* lf_handle_p2p_connections_from_federates(void*);

/**
 * @brief Send a latest tag confirmed (LTC) signal to the RTI.
 *
 * This avoids the send if an equal or later LTC has previously been sent.
 *
 * This function assumes the caller holds the mutex lock
 * on the top-level environment.
 *
 * @param tag_to_send The tag to send.
 */
void lf_latest_tag_confirmed(tag_t);

/**
 * @brief Parse the address of the RTI and store them into the global federation_metadata struct.
 * @return a parse_rti_code_t indicating the result of the parse.
 */
parse_rti_code_t lf_parse_rti_addr(const char* rti_addr);

/**
 * @brief Reset the status fields on network input ports to unknown or absent.
 *
 * This will reset to absent if the last_known_status_tag field of the port
 * is greater than or equal to the current tag of the top-level environment.
 * This should be overriden to present if an event gets scheduled.
 * Otherwise, set the status to unknown.
 * @note This function must be called at the beginning of each
 *  logical time.
 */
void lf_reset_status_fields_on_input_port_triggers();

/**
 * @brief Send a message to another federate.
 *
 * This function is used for physical connections
 * between federates. If the socket connection to the remote federate or the RTI has been broken,
 * then this returns -1 without sending. Otherwise, it returns 0.
 *
 * This method assumes that the caller does not hold the lf_outbound_socket_mutex lock,
 * which it acquires to perform the send.
 *
 * @param message_type The type of the message being sent (currently only MSG_TYPE_P2P_MESSAGE).
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The name of the next destination in string format (for reporting).
 * @param length The message length.
 * @param message The message.
 * @return 0 if the message has been sent, -1 otherwise.
 */
int lf_send_message(int message_type, unsigned short port, unsigned short federate, const char* next_destination_str,
                    size_t length, unsigned char* message);

/**
 * @brief Send information about connections to the RTI.
 *
 * This is a generated function that sends information about connections between this federate
 * and other federates where messages are routed through the RTI. Currently, this
 * only includes logical connections when the coordination is centralized. This
 * information is needed for the RTI to perform the centralized coordination.
 * @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
 */
void lf_send_neighbor_structure_to_RTI(int);

/**
 * @brief Send a next event tag (NET) signal.
 *
 * If this federate depends on upstream federates or sends data to downstream
 * federates, then send to the RTI a NET, which will give the tag of the
 * earliest event on the event queue, or, if the queue is empty, the timeout
 * time, or, if there is no timeout, FOREVER.
 *
 * If there are network outputs that
 * depend on physical actions, then insert a dummy event to ensure this federate
 * advances its tag so that downstream federates can make progress.
 *
 * A NET is a promise saying that, absent network inputs, this federate will
 * not produce an output message with tag earlier than the NET value.
 *
 * If there are upstream federates, then after sending a NET, this will block
 * until either the RTI grants the advance to the requested time or the wait
 * for the response from the RTI is interrupted by a change in the event queue
 * (e.g., a physical action triggered or a network message arrived).
 * If there are no upstream federates, then it will not wait for a TAG
 * (which won't be forthcoming anyway) and returns the earliest tag on the event queue.
 *
 * If the federate has neither upstream nor downstream federates, then this
 * returns the specified tag immediately without sending anything to the RTI.
 *
 * If there is at least one physical action somewhere in the federate that can
 * trigger an output to a downstream federate, then the NET is required to be
 * less than the current physical time. If physical time is less than the
 * earliest event in the event queue (or the event queue is empty), then this
 * function will insert a dummy event with a tag equal to the current physical
 * time (and a microstep of 0). This will enforce advancement of tag for this
 * federate and causes a NET message to be sent repeatedly as physical time
 * advances with the time interval between messages controlled by the target
 * parameter coordination-options: {advance-message-interval timevalue}. It will
 * stop creating dummy events if and when its event queue has an event with a
 * timestamp less than physical time.
 *
 * If wait_for_reply is false, then this function will simply send the
 * specified tag and return that tag immediately. This is useful when a
 * federate is shutting down and will not be sending any more messages at all.
 *
 * In all cases, this returns either the specified tag or
 * another tag when it is safe to advance logical time to the returned tag.
 * The returned tag may be less than the specified tag if there are upstream
 * federates and either the RTI responds with a lesser tag or
 * the wait for a response from the RTI is interrupted by a
 * change in the event queue.
 *
 * This function is used in centralized coordination only.
 *
 * This function assumes the caller holds the mutex lock.
 *
 * @param env The environment of the federate
 * @param tag The tag.
 * @param wait_for_reply If true, wait for a reply.
 */
tag_t lf_send_next_event_tag(environment_t* env, tag_t tag, bool wait_for_reply);

/**
 * @brief Send a port absent message.
 *
 * This informs the remote federate that it will not receive a message with tag less than the
 * current tag of the specified environment delayed by the additional_delay.
 *
 * @param env The environment from which to get the current tag.
 * @param additional_delay The after delay of the connection or NEVER if none.
 * @param port_ID The ID of the receiving port.
 * @param fed_ID The fed ID of the receiving federate.
 */
void lf_send_port_absent_to_federate(environment_t* env, interval_t additional_delay, unsigned short port_ID,
                                     unsigned short fed_ID);

/**
 * @brief Send a MSG_TYPE_STOP_REQUEST message to the RTI.
 *
 * The payload is the specified tag plus one microstep. If this federate has previously
 * received a stop request from the RTI, then do not send the message and
 * return 1. Return -1 if the socket is disconnected. Otherwise, return 0.
 * @return 0 if the message is sent.
 */
int lf_send_stop_request_to_rti(tag_t stop_tag);

/**
 * @brief Send a tagged message to the specified port of the specified federate.
 *
 * The tag will be the current tag of the specified environment delayed by the specified additional_delay.
 * If the delayed tag falls after the timeout time, then the message is not sent and -1 is returned.
 * The caller can reuse or free the memory storing the message after this returns.
 *
 * If the message fails to send (e.g. the socket connection is broken), then the
 * response depends on the message_type.  For MSG_TYPE_TAGGED_MESSAGE, the message is
 * supposed to go via the RTI, and failure to communicate with the RTI is a critical failure.
 * In this case, the program will exit with an error message. If the message type is
 * MSG_TYPE_P2P_TAGGED_MESSAGE, then the failure is not critical. It may be due to the
 * remote federate having exited, for example, because its safe-to-process offset led it
 * to believe that there were no messages forthcoming.  In this case, on failure to send
 * the message, this function returns -11.
 *
 * This method assumes that the caller does not hold the lf_outbound_socket_mutex lock,
 * which it acquires to perform the send.
 *
 * @param env The environment from which to get the current tag.
 * @param additional_delay The after delay on the connection or NEVER is there is none.
 * @param message_type The type of the message being sent. Currently can be
 *  MSG_TYPE_TAGGED_MESSAGE for messages sent via the RTI or MSG_TYPE_P2P_TAGGED_MESSAGE
 *  for messages sent directly between federates.
 * @param port The ID of the destination port.
 * @param federate The ID of the destination federate.
 * @param next_destination_str The next destination in string format (RTI or federate)
 *  (used for reporting errors).
 * @param length The message length.
 * @param message The message.
 * @return 0 if the message has been sent, 1 otherwise.
 */
int lf_send_tagged_message(environment_t* env, interval_t additional_delay, int message_type, unsigned short port,
                           unsigned short federate, const char* next_destination_str, size_t length,
                           unsigned char* message);

/**
 * @brief Set the federation_id of this federate.
 * @param fid The federation ID.
 */
void lf_set_federation_id(const char* fid);

#ifdef FEDERATED_DECENTRALIZED
/**
 * @brief Spawn a thread to iterate through STAA structs.
 *
 * This will set their associated ports absent
 * at an offset if the port is not present with a value by a certain physical time.
 */
void lf_spawn_staa_thread(void);
#endif

/**
 * @brief Wait until inputs statuses are known up to and including the specified level.
 *
 * Specifically, wait until the specified level is less that the max level allowed to
 * advance (MLAA).
 * @param env The environment (which should always be the top-level environment).
 * @param level The level to which we would like to advance.
 */
void lf_stall_advance_level_federation(environment_t* env, size_t level);

/**
 * @brief Version of lf_stall_advance_level_federation() that assumes the caller holds the mutex lock.
 * @param level The level to which we would like to advance.
 */
void lf_stall_advance_level_federation_locked(size_t level);

/**
 * @brief Synchronize the start with other federates via the RTI.
 *
 * This assumes that a connection to the RTI is already made
 * and _lf_rti_socket_TCP is valid. It then sends the current logical
 * time to the RTI and waits for the RTI to respond with a specified
 * time. It starts a thread to listen for messages from the RTI.
 */
void lf_synchronize_with_other_federates();

/**
 * @brief Update the max level allowed to advance (MLAA).
 *
 * If the specified tag is greater than the current_tag of the top-level environment
 * (or equal and is_provisional is false), then set the MLAA to INT_MAX and return.
 * This removes any barriers on execution at the current tag due to network inputs.
 * Otherwise, set the MLAA to the minimum level over all (non-physical) network input ports
 * where the status of the input port is not known at that current_tag.
 *
 * This function assumes that the caller holds the mutex.
 *
 * @param tag The latest TAG or PTAG received by this federate.
 * @param is_provisional Whether the tag was provisional.
 * @return True if the MLAA changed.
 */
bool lf_update_max_level(tag_t tag, bool is_provisional);

#ifdef FEDERATED_DECENTRALIZED
/**
 * @brief Return the physical time that we should wait until before advancing to the specified tag.
 *
 * This function adds the STA offset (STP_offset parameter) to the time of the specified tag unless
 * the tag is the starting tag (it is always safe to advance to the starting tag). It also avoids
 * adding the STA offset if all network input ports are known at least up to one microstep earlier
 * than the specified tag.
 *
 * This function assumes that the caller holds the environment mutex.
 * @param time The specified time.
 */
instant_t lf_wait_until_time(tag_t tag);
#endif // FEDERATED_DECENTRALIZED

#endif // FEDERATE_H
