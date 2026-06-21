import { expect, test } from "vitest";
import { handleKey, type TuningView } from "./keymap";

const view: TuningView = { bprTarget: 1.0, kp: 20, ki: 20 };

test("] raises bpr target by 0.05 (absolute command + new state)", () => {
  const r = handleKey("]", view);
  expect(r).toEqual({ type: "command", command: "bpr 1.05", tuning: { bprTarget: 1.05, kp: 20, ki: 20 } });
});

test("[ lowers bpr target by 0.05 with clean rounding", () => {
  const r = handleKey("[", view);
  expect(r).toEqual({ type: "command", command: "bpr 0.95", tuning: { bprTarget: 0.95, kp: 20, ki: 20 } });
});

test(". raises kp, , lowers kp", () => {
  expect(handleKey(".", view)).toMatchObject({ command: "kp 21" });
  expect(handleKey(",", view)).toMatchObject({ command: "kp 19" });
});

test("' raises ki, ; lowers ki", () => {
  expect(handleKey("'", view)).toMatchObject({ command: "ki 21" });
  expect(handleKey(";", view)).toMatchObject({ command: "ki 19" });
});

test("a and p are commands; l and q are local actions", () => {
  expect(handleKey("a", view)).toEqual({ type: "command", command: "auto", tuning: view });
  expect(handleKey("p", view)).toEqual({ type: "command", command: "params", tuning: view });
  expect(handleKey("l", view)).toEqual({ type: "action", action: "relabel" });
  expect(handleKey("q", view)).toEqual({ type: "action", action: "quit" });
});

test("unmapped key does nothing", () => {
  expect(handleKey("z", view)).toEqual({ type: "none" });
});
