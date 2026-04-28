# Exp.5 Drake URDFs

These lightweight URDFs mirror the DH chains used by the v6 SBF robot JSON
files. They use primitive visual/collision geometry so Exp.5 scenes can be
loaded directly by Drake without relying on external mesh packages.

Complete upstream robot description packages are stored under
`cpp/v6/data/urdf/upstream/`.

Use the lightweight `*_exp5.urdf` files for the current Exp.5 pipeline when you
want a self-contained Drake import with minimal dependencies. Use the upstream
packages when you need the full Panda or UR5 mesh-based descriptions.
