import json
from pathlib import Path

ALIASES = {
    "leftTrigger": "lt",
    "rightTrigger": "rt",
    "leftStick": "lsb",
    "rightStick": "rsb",
    "leftBumper": "lb",
    "rightBumper": "rb",
}

controller_map = json.loads((Path(__file__).parent / "map.json").read_text())
driver = controller_map["driver"].items()
operator = controller_map["operator"].items()

TEMPLATE = """<!DOCTYPE html>
<html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Controller Maps</title>
        <script src="https://cdn.jsdelivr.net/npm/@unocss/runtime"></script>
        <link
        rel="stylesheet"
        href="https://cdn.jsdelivr.net/npm/@picocss/pico@2/css/pico.min.css"
    >
    </head>
    <body>
        <div class="w-full flex justify-evenly space-x-36">
            <div>
                <h2 class="text-7xl text-center">Driver</h2>
                <img src="./labelled.png" class="w-full text-red-500"></img>
                <table>
                    <thead>
                        <tr>
                            <th scope="col">Button</th>
                            <th scope="col">Command</th>
                        </tr>
                    </thead>
                    <tbody>
                    {}
                    </tbody>
                </table>
            </div>
            <div>
                <h2 class="text-7xl text-center">Operator</h2>
                <img src="./labelled.png" class="w-full"></img>
                <table>
                    <thead>
                        <tr>
                            <th scope="col">Button</th>
                            <th scope="col">Command</th>
                        </tr>
                    </thead>
                    <tbody>
                    {}
                    </tbody>
                </table>
            </div>
        </div>
    </body>
</html>
""".format(
    "\n".join(
        map(
            lambda x: f"<tr><td>{ALIASES.get(x[0], x[0])}</td><td>{x[1]}</td></tr>",
            driver,
        )
    ),
    "\n".join(
        map(
            lambda x: f"<tr><td>{ALIASES.get(x[0], x[0])}</td><td>{x[1]}</td></tr>",
            operator,
        )
    ),
)

(Path(__file__).parent / "index.html").write_text(TEMPLATE)
