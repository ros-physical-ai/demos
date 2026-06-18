# Contributing

## Linting & Pre-commit

This repository uses [pre-commit](https://pre-commit.com/) to enforce consistent code quality. The following hooks are configured:

- **General**: trailing whitespace, end-of-file fixer, YAML/XML validation, large file check, merge conflict markers
- **Python**: [Ruff](https://docs.astral.sh/ruff/) for linting and formatting
- **Shell**: [ShellCheck](https://www.shellcheck.net/) for static analysis
- **YAML/Markdown**: [Prettier](https://prettier.io/) for formatting
- **CMake**: [cmake-lint](https://cmake-format.readthedocs.io/) for CMakeLists.txt files

### Setup

Install the git hooks so they run automatically on every commit:

```bash
pre-commit install
```

With Pixi: `pixi run -e default pre-commit install`

### Usage

Hooks will run automatically on staged files when you `git commit`. To run all hooks on all files manually:

```bash
pre-commit run --all-files
```

With Pixi: `pixi run lint`
