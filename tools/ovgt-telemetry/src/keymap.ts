export interface TuningView {
  bprTarget: number;
  kp: number;
  ki: number;
}

export type KeyResult =
  | { type: "command"; command: string; tuning: TuningView }
  | { type: "action"; action: "relabel" | "quit" }
  | { type: "none" };

const round2 = (n: number): number => Math.round(n * 100) / 100;

export function handleKey(input: string, t: TuningView): KeyResult {
  switch (input) {
    case "[": {
      const v = round2(t.bprTarget - 0.05);
      return { type: "command", command: `bpr ${v}`, tuning: { ...t, bprTarget: v } };
    }
    case "]": {
      const v = round2(t.bprTarget + 0.05);
      return { type: "command", command: `bpr ${v}`, tuning: { ...t, bprTarget: v } };
    }
    case ",": {
      const v = t.kp - 1;
      return { type: "command", command: `kp ${v}`, tuning: { ...t, kp: v } };
    }
    case ".": {
      const v = t.kp + 1;
      return { type: "command", command: `kp ${v}`, tuning: { ...t, kp: v } };
    }
    case ";": {
      const v = t.ki - 1;
      return { type: "command", command: `ki ${v}`, tuning: { ...t, ki: v } };
    }
    case "'": {
      const v = t.ki + 1;
      return { type: "command", command: `ki ${v}`, tuning: { ...t, ki: v } };
    }
    case "a":
      return { type: "command", command: "auto", tuning: t };
    case "p":
      return { type: "command", command: "params", tuning: t };
    case "l":
      return { type: "action", action: "relabel" };
    case "q":
      return { type: "action", action: "quit" };
    default:
      return { type: "none" };
  }
}
