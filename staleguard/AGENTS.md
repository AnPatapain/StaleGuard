## Communication and reporting rules

Write for a researcher who understands the project but does not want to decode implementation jargon.

For every experiment report:

1. **Start with a plain-English summary (maximum 200 words):**
   - What did you implement?
   - How does it work physically? Use a concrete UAV example.
   - What did the experiments show? Include actual numbers.
   - What is wrong, missing, or uncertain?
   - What should we do next?

2. **Explain the experiment before the implementation.**
   Describe what the UAV does, what changes, and what constitutes success or failure.

3. **Use simple language.**
   Avoid unnecessary jargon. If a technical term is essential, explain it immediately in parentheses.

4. **Keep the main report short.**
   Aim for 1–2 pages. Put implementation details, file inventories, test logs, hashes, and reproduction commands in a separate technical appendix.

5. **Use tables for experimental results.**
   Clearly distinguish observed facts, assumptions, and recommendations.

6. **Never sacrifice scientific accuracy for simplicity.**
   Report important failures, limitations, and methodological concerns, but explain why they matter in practical terms.

**Writing test:** A reader should understand the mission, results, and next decision without opening source code or decoding technical terminology.

## How to make the explanation readable

- Write connected paragraphs. Explain what happens, why it happens, and what it means for the experiment. Avoid strings of short factual statements that leave the reader to reconstruct the relationships.
- Describe the aircraft's motion before naming a function, variable, data structure, or numerical method. Keep code-level language in the appendix unless it is necessary to understand the result.
- Use a concrete example to explain an unfamiliar mechanism. Clearly label invented coordinates as illustrative; identify measured examples by their recorded scenario and do not mix the two.
- Label the evidence before interpreting it: an actual PyBullet flight, a model prediction, a diagnostic timing benchmark, and an official mission experiment answer different questions. Distinguish warmed prediction time from the first call and from total mission-run time.
- Prefer an explanation such as “waiting another half-second moves the turn farther east, so the UAV can miss the same checkpoint” to an unexplained phrase such as “execution-state-relative reference semantics.” More words are justified when they save the reader from decoding the sentence.
- Before finishing, read only the main report. It must explain the mission, the meaning of success and failure, the measured results, the limitations, and the next decision. If the reader needs the appendix to understand any of these, revise the main report rather than adding more technical detail to it.
