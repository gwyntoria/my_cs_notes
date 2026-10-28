# Repository Guidelines

## Change Boundaries

- Before editing, run `git status --short` and inspect existing changes in the files you need to edit. Preserve staged, unstaged, and untracked work.
- Limit edits to the requested task. Leave unrelated content, formatting, and asset locations unchanged; review or explanation requests alone do not authorize edits.
- Before finishing, inspect the final diff to confirm that your changes stay within scope and preserve existing work.

## Image File Path

Save note images under `./assets/<filename>/`, where `./assets` is the assets subdirectory beside the owning Markdown file and `<filename>` matches that file without its `.md` extension. Reference the image from the note with a relative Markdown path.

## Code File Path

Save code accompanying a note under `./assets/code/<filename>/`, where `./assets` is the assets subdirectory beside the owning Markdown file and `<filename>` matches that file without its `.md` extension. Reference the code from the note with a relative Markdown path.

## References and Footnotes

For Markdown notes with external references:

- Cite sources in the body with numeric footnote markers, for example `[^1]`. Assign numbers by first appearance and reuse the same number for repeated citations of the same source.
- Add a `## References` section at the end of the note/article. List footnote definitions in numeric order, using the form `[^1]: Author, [Title](URL), year.`, with a short description when useful.
- Do not use named footnote markers such as `[^active-object]`.
