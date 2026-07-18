# Git Workflow

## Branches

- `main` is the deployable UAV-side baseline.
- Use `feature/<name>` for features, `fix/<name>` for corrections, and
  `experiment/<name>` for research variants.
- Keep experiment-specific configuration and results separate from the runtime
  defaults used by `main`.

## Commits

Use concise imperative subjects with one of these prefixes:

- `feat:` user-visible or runtime capability
- `fix:` defect correction
- `docs:` documentation only
- `test:` test-only change
- `build:` build or dependency change
- `chore:` repository maintenance

Do not commit build products, virtual environments, runtime logs, flight bags,
API credentials, private keys, or machine-local overrides. Review staged files
with `git diff --cached --stat` and `git diff --cached` before every commit.

## Verification

Run checks in proportion to the changed subsystem. At minimum, run the focused
unit tests for touched Python components and validate ROS launch/config syntax
when those files change. Flight tests are never a substitute for non-flight
validation and must follow the normal onboard safety procedure.

## Third-Party Code

Keep imported upstream code under an explicit `upstream/` directory when
possible. Record repository, branch, commit, exclusions, and local patches in a
source manifest. Do not mix local adapters into an upstream source tree.

## Large Files

No committed file may exceed GitHub's 100 MB limit. Before adding a new model,
bag, dataset, or large binary, decide whether it belongs in Git LFS, release
assets, or external storage. Do not add it directly and rewrite history later.

## Secrets

Runtime secrets must come from environment files installed outside the working
tree or from the service manager. Example files may contain variable names and
safe placeholders only. Rotate any credential that has previously been
committed or shared as source text.
