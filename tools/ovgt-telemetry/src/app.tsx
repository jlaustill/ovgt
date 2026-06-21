import { Box, Text, useApp, useInput } from "ink";
import React from "react";
import { formatTelemetry } from "./format";
import { handleKey, type TuningView } from "./keymap";
import type { SettleEvent, TelemetrySample } from "./types";

export interface AppProps {
  sample?: TelemetrySample;
  settle?: SettleEvent;
  logs: string[];
  sessionLabel: string;
  status: string;
  onCommand: (command: string) => void;
  onRelabel: (label: string) => void;
}

export function App(props: AppProps): React.ReactElement {
  const { sample, settle, logs, sessionLabel, status, onCommand, onRelabel } = props;
  const { exit } = useApp();
  const [tuning, setTuning] = React.useState({ kp: 20, ki: 20 });
  const [vaneBuf, setVaneBuf] = React.useState("");
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

    if (/^[0-9]$/.test(input)) {
      setVaneBuf((b) => (b + input).slice(0, 3));
      return;
    }
    if (key.return) {
      if (vaneBuf) {
        onCommand(String(Math.min(100, parseInt(vaneBuf, 10))));
        setVaneBuf("");
      }
      return;
    }
    if (key.escape) {
      setVaneBuf("");
      return;
    }

    const view: TuningView = { bprTarget: sample?.bpr_target ?? 1.0, kp: tuning.kp, ki: tuning.ki };
    const res = handleKey(input, view);
    if (res.type === "command") {
      onCommand(res.command);
      setTuning({ kp: res.tuning.kp, ki: res.tuning.ki });
    } else if (res.type === "action") {
      if (res.action === "relabel") setLabelMode(true);
      else if (res.action === "quit") exit();
    }
  });

  const lines = sample ? formatTelemetry(sample) : ["(waiting for telemetry…)"];

  return (
    <Box flexDirection="column">
      <Box borderStyle="round" flexDirection="column" paddingX={1}>
        <Text bold>
          OVGT [{sessionLabel}] {status} kp {tuning.kp} ki {tuning.ki}
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
        {vaneBuf && <Text color="yellow">vane → {vaneBuf} (Enter to send)</Text>}
        {labelMode && <Text color="green">label → {labelBuf}_ (Enter to save, Esc cancel)</Text>}
      </Box>
      <Box flexDirection="column" paddingX={1}>
        {logs.slice(-6).map((l, i) => (
          <Text key={i} dimColor>
            {l}
          </Text>
        ))}
      </Box>
      <Text dimColor>[ ] bpr  , . kp  ; ' ki  a auto  p params  digits+Enter vane  l label  q quit</Text>
    </Box>
  );
}
