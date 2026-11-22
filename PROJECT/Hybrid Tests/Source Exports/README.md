# Source exports

Readable code blocks extracted from each `.mlx`, without changing its equations. They support search and code review on GitHub; the `.mlx` in the parent directory retains its original layout and embedded outputs.

Before running an export, check its Current Folder and dependencies. In particular, V8 looks for `Ensayos` relative to `pwd`. Do not recursively add this directory and every historical version to the path. Prefer the parent directory's `.mlx` for execution.
