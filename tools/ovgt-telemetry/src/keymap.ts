// The TUI never mutates the controller — tuning is compile-time (edit + reflash).
// So the only keys are local actions: label the session, or quit.
export type KeyResult =
  | { type: "action"; action: "relabel" | "quit" }
  | { type: "none" };

export function handleKey(input: string): KeyResult {
  switch (input) {
    case "l":
      return { type: "action", action: "relabel" };
    case "q":
      return { type: "action", action: "quit" };
    default:
      return { type: "none" };
  }
}
