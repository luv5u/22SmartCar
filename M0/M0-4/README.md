##参数说明

| 参数 | 必填 | 默认值 | 说明 |
| :-- | :--: | :-- | :-- |
| `--config <path>` | **是** | 无 | 任务配置文件路径，支持 `.yaml` / `.yml` / `.json`。扩展名决定解析方式。 |
| `--timeout <float>` | 否 | 配置文件中的 `timeout` | 全局超时（秒），覆盖配置文件里的 `timeout`。两者都不提供时不启用超时。 |
| `--report <path>` | 否 | `report.json` | 报告输出路径。相对路径基于当前工作目录；所在目录需已存在。 |
| `--seed <int>` | 否 | 不固定 | 随机种子。相同 `seed` 且相同配置下，成功/失败序列可复现。 |

## 运行示例

### 示例 1：基本运行（固定随机种子）
演示最常用用法：加载 YAML 配置，固定 `seed` 以便复现，报告写到默认 `report.json`。

```bash
python3 scheduler.py --config tasks_demo.yaml --seed 42
```

预期：
- 控制台实时彩色输出 任务的执行状态
- 当前目录生成 `report.json`，包含全部任务及总耗时

### 示例 2：超时中断（演示 `--timeout` 与自定义 `--report`）
演示 `--timeout` 覆盖配置中的超时，并用 `--report` 指定独立的报告文件，避免覆盖示例 1 的结果。

```bash
python3 scheduler.py --config tasks_demo.yaml --timeout 1 --seed 42 --report report_timeout.json
```

预期：
- 总耗时超过 1 秒即中断，触发任务及未执行任务状态为 `TIMEOUT`
- 控制台输出 `[TIMEOUT] ...` 提示
- `report_timeout.json` 中 `"timeout": true`，未执行任务也出现在 `tasks` 列表中

### 示例 3：使用 JSON 配置 + 子目录报告
演示 `.json` 配置同样支持，`--report` 可指向子目录（需先创建目录）。

```bash
mkdir -p reports
python3 scheduler.py --config tasks_demo.json --report reports/report.json --seed 7
```

预期：
- 与 YAML 配置行为一致
- 报告写入 `reports/report.json`

