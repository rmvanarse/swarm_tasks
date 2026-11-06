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

    def __init__(self):
        self._queue = queue.Queue()

    def call_back(self, data, addr):
        self._queue.put(data.decode())

    def get_current_data(self):
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def set_data(self):
        """No-op: get_current_data() already dequeued the message, so
        there's nothing left to clear. Kept only so main.py's existing
        callBack.set_data() call sites don't need to change."""

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