---

# Coding & Engineering Guidelines (Karpathy + Ponytail)

Behavioral guidelines to reduce common LLM coding mistakes and prevent over-engineering.

## 1. Think Before Coding
**Don't assume. Don't hide confusion. Surface tradeoffs.**
- State your assumptions explicitly. If uncertain, ask before writing code.
- If multiple interpretations exist, present them instead of picking silently.
- If a simpler approach exists, say so. Push back when warranted.
- Trace the real flow and read the code the change touches before acting.

## 2. Simplicity & YAGNI First (Ponytail Decision Ladder)
**The best code is the code you never wrote. Write only what the task needs.**
Before writing new code, check in this exact order:
1. **Do nothing / Delete:** Can this be solved by removing code or changing config instead of adding code? Deletion beats addition.
2. **Native / Standard Library:** Can built-in language features, standard libraries, or native platform/browser APIs handle this?
3. **Existing Dependencies:** Can libraries or helpers already in the project solve this?
4. **Minimal New Code:** Write the smallest complete change that solves the immediate problem.
- No speculative features, single-use abstractions, or unrequested configurability.
- Never cut validation, error handling, security, or accessibility just to save lines.

## 3. Surgical Changes
**Touch only what you must. Clean up only your own mess.**
- When editing existing code, do not "improve" adjacent code, comments, or formatting.
- Match existing style, even if you would do it differently.
- Remove imports/variables/functions that YOUR changes made unused. Do not remove pre-existing dead code unless asked.
- Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution
**Define success criteria. Loop until verified.**
- Transform tasks into verifiable goals (e.g., write a reproducing test -> make it pass).
- For multi-step tasks, state a brief plan with verification checks for each step.
- End implementations by noting any skipped edge cases, unchecked items, or trade-offs.
