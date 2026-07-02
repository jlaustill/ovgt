import { Box, Text, useApp, useInput } from "ink";
import React from "react";
import { formatTelemetry } from "./format";
import { handleKey } from "./keymap";
import type { SettleEvent, TelemetrySample } from "./types";

export interface AppProps {
  sample?: TelemetrySample;
  settle?: SettleEvent;
  logs: string[];
  sessionLabel: string;
  status: string;
  onRelabel: (label: string) => void;
}

// The TUI is read-only for the truck: it displays telemetry and lets you label
// the session, nothing more. Tuning is compile-time (edit firmware + reflash) by
// design — there are deliberately no keys that change the controller live.
export function App(props: AppProps): React.ReactElement {
  const { sample, settle, logs, sessionLabel, status, onRelabel } = props;
  const { exit } = useApp();
  const [labelMode, setLabelMode] = React.useState(false);
  const [labelBuf, setLabelBuf] = React.useState("");

  useInput((input, key) => {
    if (labelMode) {
      if (key.return) {
        onRelabel(labelBuf);
        setLabelMode(false);
        setLabelBuf("");
      } else if (key.escape) {
        setLabelMode(false);
        setLabelBuf("");
      } else if (key.backspace || key.delete) {
        setLabelBuf((b) => b.slice(0, -1));
      } else if (input) {
        setLabelBuf((b) => b + input);
      }
      return;
    }

    const res = handleKey(input);
    if (res.type === "action") {
      if (res.action === "relabel") setLabelMode(true);
      else if (res.action === "quit") exit();
    }
  });

  const lines = sample ? formatTelemetry(sample) : ["(waiting for telemetry…)"];

  return (
    <Box flexDirection="column">
      <Box borderStyle="round" flexDirection="column" paddingX={1}>
        <Text bold>
          OVGT [{sessionLabel}] {status}
        </Text>
        {lines.map((l, i) => (
          <Text key={i}>{l}</Text>
        ))}
        {settle && (
          <Text color="cyan">
            last settle: τ {settle.tau_s.toFixed(1)}s step {settle.step_c.toFixed(1)}C slope{" "}
            {settle.cot_slope_c_s.toFixed(2)}C/s
          </Text>
        )}
        {labelMode && <Text color="green">label → {labelBuf}_ (Enter to save, Esc cancel)</Text>}
      </Box>
      <Box flexDirection="column" paddingX={1}>
        {logs.slice(-6).map((l, i) => (
          <Text key={i} dimColor>
            {l}
          </Text>
        ))}
      </Box>
      <Text dimColor>l label  q quit  ·  tuning is compile-time (edit firmware + reflash)</Text>
    </Box>
  );
}
