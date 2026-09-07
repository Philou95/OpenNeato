const assert = require("node:assert/strict");
const { readFileSync } = require("node:fs");
const { test } = require("node:test");
const vm = require("node:vm");
const source = readFileSync("custom_components/openneato/www/openneato-replay-card.js", "utf8");

function load() {
    const elements = new Map();
    const context = vm.createContext({
        HTMLElement: class {},
        window: {},
        console: { info() {}, debug() {} },
        customElements: {
            define(name, value) {
                if (elements.has(name)) throw new Error("Already defined");
                elements.set(name, value);
            },
        },
    });
    // Each module load has its own lexical bindings, like two browser loaders.
    const execute = () => vm.runInContext(`(() => { ${source}\nreturn Session; })()`, context);
    return { context, elements, execute, Session: execute() };
}

test("card registration survives duplicate loading without duplicate picker entries", () => {
    const card = load();
    card.execute();
    assert.equal(card.elements.size, 1);
    assert.equal(card.context.window.customCards.length, 1);
});

test("replay interpolates position and heading across the 180-degree boundary", () => {
    const { Session } = load();
    const session = new Session({ path: [0, 0, 179, 0, 2, 4, -179, 10] });
    const mid = session.interpolate(5);
    assert.equal(mid.x, 1);
    assert.equal(mid.y, 2);
    assert.equal(Math.abs(mid.t), 180);
    assert.equal(session.poseCountUpTo(0), 1);
    assert.equal(session.poseCountUpTo(10), 2);
});

test("empty sessions are safe and coverage is revealed in timestamp order", () => {
    const { Session } = load();
    const session = new Session({ coverage: [3, 4, 20, 1, 2, 10] });
    assert.equal(session.interpolate(0), null);
    assert.equal(session.poseCountUpTo(100), 0);
    assert.deepEqual(Array.from(session.coverage), [1, 2, 10, 3, 4, 20]);
});
