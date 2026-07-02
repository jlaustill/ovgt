import { expect, test } from "vitest";
import { handleKey } from "./keymap";

test("l requests relabel, q requests quit", () => {
  expect(handleKey("l")).toEqual({ type: "action", action: "relabel" });
  expect(handleKey("q")).toEqual({ type: "action", action: "quit" });
});

test("tuning keys are gone — they do nothing (no runtime config)", () => {
  for (const key of ["[", "]", ",", ".", ";", "'", "a", "p", "5", "z"]) {
    expect(handleKey(key)).toEqual({ type: "none" });
  }
});
