# M20 Navigation Data Flywheel Design

- Date: 2026-07-18
- Session id: 2026-07-18_0030_m20-navigation-data-flywheel-design
- Project: MeloLong DimOS navigation
- Workspace: `/tmp/dimos-official-dannav-bool-fix`
- Task: Document the M20 navigation data-collection and offline-evaluation flywheel.
- Status: completed
- Branch: `wd/m20-dan-nav-upgrade`

## Request Summary

Write an R&D document that prioritizes reproducible navigation data collection and offline evaluation before model training.

## Work Done

- Added `docs/development/m20-navigation-data-flywheel.md`.
- Added the document to the Development navigation in `docs/docs.json`.
- Defined the run manifest, stream contract, event taxonomy, dataset layout, metrics, scenario sets, release gates, and staged delivery plan.
- Verified `docs/docs.json` with Python JSON parsing and ran `git diff --check`.

## Current State

- Documentation is ready to commit and push.
- The Mint documentation tool is not installed in this worktree, so site validation was not run.

## Resume Instructions

- Implement M1: run manifest schema, outcome taxonomy, and dataset directory creator.

## Open Questions

- Metric thresholds will be set after an initial baseline dataset exists.
