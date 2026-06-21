"""Isolate the lietorch Sim3 group-op crash: fresh tensors vs shared-memory-backed."""
import torch
import lietorch

print("[lt] torch", torch.__version__, "cuda", torch.cuda.is_available(), flush=True)

# Case A: fresh identity Sim3 group ops (inv, mul, act, retr) on the GPU.
print("[lt] A: fresh Sim3.inv() * Sim3 ...", flush=True)
a = lietorch.Sim3.Identity(1, device="cuda")
b = lietorch.Sim3.Identity(1, device="cuda")
c = a.inv() * b
torch.cuda.synchronize()
print("[lt] A: inv*mul ok", flush=True)
X = torch.randn(1, 3, device="cuda")
_ = c.act(X)
torch.cuda.synchronize()
print("[lt] A: act ok", flush=True)
tau = torch.zeros(1, 7, device="cuda")
_ = c.retr(tau)
torch.cuda.synchronize()
print("[lt] A: retr ok  => fresh group ops WORK", flush=True)

# Case B: the SharedKeyframes layout — Sim3 wrapping a slice of a share_memory_() buffer.
print("[lt] B: Sim3 over a share_memory_() buffer slice ...", flush=True)
buf = torch.zeros(8, 1, lietorch.Sim3.embedded_dim, device="cuda").share_memory_()
buf[:] = lietorch.Sim3.Identity(1, device="cuda").data  # broadcast identity into every slot
Tk = lietorch.Sim3(buf[0])
Tf = lietorch.Sim3(buf[1])
print("[lt] B: wrapped ok; calling inv()*  ...", flush=True)
d = Tk.inv() * Tf
torch.cuda.synchronize()
print("[lt] B: shared-memory inv*mul ok", flush=True)

# Case C: same but cloned to a contiguous, non-shared tensor first (candidate fix).
print("[lt] C: clone slice then group op ...", flush=True)
Tk2 = lietorch.Sim3(buf[0].clone())
Tf2 = lietorch.Sim3(buf[1].clone())
e = Tk2.inv() * Tf2
torch.cuda.synchronize()
print("[lt] C: cloned inv*mul ok", flush=True)

print("[lt] ALL LIETORCH CASES PASSED", flush=True)
