import queue
import socketserver
import threading

class CallBack:
    """Hands received datagrams to main.py's poll loop, in arrival order.

    Backed by a Queue instead of a single overwritable slot: UDPReceiver
    runs ThreadingMixIn, so a new thread fires call_back() for every
    packet, independent of whether main.py has finished handling the
    last one yet. A single `self.data` slot meant a second command
    (e.g. a "goal" for bots 4,5,6) arriving while the first was still
    being read/dispatched got overwritten, and the unconditional
    set_data() afterwards then wiped it out -- it never reached main.py's
    if/elif dispatch at all, silently, with nothing printed to say a
    command was dropped. A queue means every packet is eventually
    delivered and in the order it arrived, no matter how fast they're
    sent relative to how long main.py takes to process one.
    """

    # A command may end with "|id=<token>"; the token is echoed in every reply.
    ID_MARK = "|id="

    def __init__(self):
        self._queue = queue.Queue()
        self._sock = None  # set by attach(); used only to send replies
        self._current = (None, None)  # (addr, command id) of the command in hand

    def attach(self, receiver):
        """Lets replies go out of the same socket the commands arrive on."""
        self._sock = receiver.socket

    def _reply(self, addr, text):
        if self._sock is None or addr is None:
            return
        try:
            self._sock.sendto(text.encode("utf-8"), addr)
        except OSError as exc:
            print(f"[UDP] reply to {addr} failed: {exc!r}")

    def call_back(self, data, addr):
        text, _, cmd_id = data.decode().partition(self.ID_MARK)
        self._queue.put((text, addr, cmd_id or None))
        if cmd_id:
            self._reply(addr, f"ACK {cmd_id} received")

    def get_current_data(self):
        try:
            text, addr, cmd_id = self._queue.get_nowait()
        except queue.Empty:
            return None
        self._current = (addr, cmd_id)
        return text

    def set_data(self, error=None):
        """Finishes the command get_current_data() handed out and tells the
        sender how it went: "ACK <id> accepted", or "REJECT <id> <reason>"
        when `error` is given. Commands sent without an id get no reply."""
        addr, cmd_id = self._current
        self._current = (None, None)
        if not cmd_id:
            return
        if error is None:
            self._reply(addr, f"ACK {cmd_id} accepted")
        else:
            reason = " ".join(str(error).split()) or "rejected"
            self._reply(addr, f"REJECT {cmd_id} {reason}")

class UDPReceiverHandler(socketserver.BaseRequestHandler):
    """Handles each incoming UDP packet by delegating to the server's callback."""

    def handle(self):
        data, sock = self.request
        # Delegate to whatever callback the server was configured with
        self.server.on_data_received(data, self.client_address)


class UDPReceiver(socketserver.ThreadingMixIn, socketserver.UDPServer):
    """
    A UDP server dedicated to receiving data.
    """

    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, callback=None, buffer_size=65536):
        super().__init__(address, UDPReceiverHandler)
        self.buffer_size = buffer_size
        self._callback = callback or self._default_callback
        self._serve_thread = None

    def _default_callback(self, data, addr):
        print(f"[UDPReceiver] {addr} -> {data.decode(errors='replace')}")

    def on_data_received(self, data: bytes, addr):
        """Called by the handler for every packet. Override or pass callback= instead."""
        print(data)
        self._callback(data, addr)

    def start(self):
        """Run the server loop in a background thread (non-blocking)."""
        self._serve_thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._serve_thread.start()
        return self

    def stop(self):
        self.shutdown()
        self.server_close()
        if self._serve_thread:
            self._serve_thread.join()