# Shared research notes

This folder is where people and coding agents (Claude, Codex, anyone else) keep research,
options and decisions that are **not yet** team rules or tested robot behavior. It exists so
a finding made in one session is not lost, and so the next session can build on it instead of
starting again.

A note here is **not permission**. [AGENTS.md](../../AGENTS.md) stays the rulebook. If a note
proposes something AGENTS.md forbids (for example, an agent starting an OpMode), the rule
wins until a mentor changes AGENTS.md.

## Notes

| Note | Question | Status |
| --- | --- | --- |
| [Agent and robot loop](agent-robot-loop.md) | Can a Mac act as the Robot Controller? How can agents observe the robot and iterate faster? | Open: options listed, most untested |
| [Logging, Hub storage and rehearsal](logging-and-rehearsal.md) | Is every run recorded, can recordings fill the Hub, and can a move driven in TeleOp become autonomous waypoints? | Trying in `pollen-hive-v10`: computer-tested, not yet run on the robot |
| [Collection motion and Pedro Pathing](collection-motion-and-pedro.md) | How does Pinpoint guide collection, what should we tune on vinyl or tiles, and where would Pedro help? | Proposed transit comparison |

## Where things belong

| Kind of content | Home |
| --- | --- |
| Rules every agent must follow | [AGENTS.md](../../AGENTS.md) |
| How to connect, build, install, download | [Desktop tools](../DESKTOP_TOOLS.md) |
| What the robot has actually done | [Robot tested behavior](../POLLEN_ROBOT_BEHAVIOR.md) and the hive guides |
| Research, options, open questions, decisions with their reasons | This folder |

When an option here is adopted, move the working instructions to the right home above and
leave one line here saying where they went.

## How to add to a note

1. **Label every claim** with how it is known. Use these exact labels so they can be searched:

   | Label | Meaning |
   | --- | --- |
   | `[doc]` | A vendor or project document says so. Give the link. |
   | `[repo]` | Read in this repository. Give the file, and the commit if it may change. |
   | `[reported]` | A team member said so. Give the name and date. |
   | `[inferred]` | Reasoning that nobody has checked. |
   | `[tested]` | Tried on the real robot or laptop. Give who, the date, and the installed build label. |

   `[doc]` and `[inferred]` are not `[tested]`. Do not upgrade a label without doing the test.
2. **Say where you ran.** A cloud session cannot reach the laptop or the robot. A local session
   can. Write which one you were.
3. **Add, do not rewrite.** Do not delete or reword another contributor's finding. If it is
   wrong or out of date, add a dated line under it that says so and why.
4. **Update the status** of an option when it changes, and add a line to that note's decision
   log with the date and who decided.
5. **Sign entries** with a date and a name (`2026-11-15, Claude cloud session`, `Codex local`,
   a person's name).
6. One topic per file, named `topic-in-lowercase.md`. Add it to the table above.

Option statuses: `Proposed`, `Trying`, `In use`, `Deferred`, `Rejected`.

## Template for a new note

```markdown
# Title

Question: one or two sentences.
Started: YYYY-MM-DD by NAME (cloud or local session). Based on branch BRANCH at COMMIT.

## Summary
## What is known
## Options
| ID | Option | Status | Evidence | Next step |
## Not verified
## Decision log
## Sources
```
