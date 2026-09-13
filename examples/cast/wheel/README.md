# `example-cast-wheel`

A 95A polyurethane tire cast onto a printed rim, and the mold that does it.

```sh
cargo run --release -p example-cast-wheel                      # the stock wheel
cargo run --release -p example-cast-wheel -- --help            # every knob
cargo run --release -p example-cast-wheel -- \
    --tire-od 150 --rim-od 120 --width 30 --cell-mm 1.0 --out ~/wheel/iter2
```

With no arguments it builds the stock wheel and **asserts an oracle** — that is
what makes it a validator rather than a demo, and what gets it run in CI instead
of merely compiled.

With arguments it is a generator: any geometry `WheelSpec` accepts, exported
wherever you point it.

⚠ The rim is **product, not tooling** (`PlugRole::Insert`). The generated
`procedure.md` tells the bencher to mold-release the cup halves and *not* the
rim, and to leave the rim in the cured tire. Following a silicone cast's
instructions here destroys the part.
