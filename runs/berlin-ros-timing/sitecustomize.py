import sys
if sys.argv[0].endswith('/gym_bridge'):
    import time, threading, gc
    from rclpy.publisher import Publisher
    def trace(msg):
        print("TIMING",round(time.time(),6),round(time.monotonic(),6),msg,flush=True)
    def clock_monitor():
        m=time.monotonic(); w=time.time()
        while True:
            time.sleep(.01)
            n=time.monotonic(); t=time.time()
            if n-m>.075 or abs(t-w-(n-m))>.03:
                trace(f"wake {n-m:.4f} wall_step {t-w-(n-m):.4f}")
            m,w=n,t
    threading.Thread(target=clock_monitor,daemon=True).start()
    orig=Publisher.publish
    last={}
    def publish(self,msg):
        if type(msg).__name__ in ("LaserScan","Odometry"):
            key=type(msg).__name__
            m=time.monotonic()
            if key in last and m-last[key]>.06:
                trace(f"{key} gap {m-last[key]:.4f} header {msg.header.stamp.sec+msg.header.stamp.nanosec/1e9:.6f}")
            last[key]=m
        return orig(self,msg)
    Publisher.publish=publish
    start={}
    def gc_log(phase,info):
        idx=info["generation"]
        if phase=="start": start[idx]=time.monotonic()
        elif time.monotonic()-start.get(idx,time.monotonic())>.04:
            trace(f"gc {idx} {time.monotonic()-start[idx]:.5f}")
    gc.callbacks.append(gc_log)
    trace("instrumentation active")
