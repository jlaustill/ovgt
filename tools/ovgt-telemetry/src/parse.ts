import type { ParsedLine, SettleEvent, TelemetrySample } from "./types";

export function parseLine(line: string): ParsedLine {
  const trimmed = line.trim();
  if (trimmed === "") return { kind: "log", text: "" };

  let obj: unknown;
  try {
    obj = JSON.parse(trimmed);
  } catch {
    return { kind: "log", text: trimmed };
  }

  if (obj === null || typeof obj !== "object") {
    return { kind: "log", text: trimmed };
  }

  const type = (obj as { type?: unknown }).type;
  if (type === "t") return { kind: "telemetry", sample: obj as TelemetrySample };
  if (type === "s") return { kind: "settle", event: obj as SettleEvent };
  return { kind: "log", text: trimmed };
}
