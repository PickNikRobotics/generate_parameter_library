# Contributing to the documentation

The book's Markdown sources live in `docs/src`. Add new chapters to `docs/src/SUMMARY.md` so they appear in the sidebar and search index.
Keep detailed reference material here; the repository README provides a quick start and links into the book. Preserve its existing heading anchors when updating links.

## Preview locally

Install [mdBook](https://rust-lang.github.io/mdBook/guide/installation.html) **0.5.4**, the version pinned in the documentation workflow. With Rust installed:

```sh
cargo install mdbook --locked --version 0.5.4
```

From the repository root:

```sh
mdbook build docs
mdbook serve docs --open
```

The generated site is written to `docs/book` and is ignored by Git. ROS dependencies are not needed to build the book.
Check chapter links and search in the preview before submitting a change.

## GitHub Pages publishing

The `Documentation` workflow builds the book on pull requests affecting documentation, pushes to `main`, and manual runs. Pull requests only build the book. Deployment is restricted to `main` in `PickNikRobotics/generate_parameter_library`.

A repository administrator must select **Settings → Pages → Build and deployment → Source → GitHub Actions** before the first deployment. If the `github-pages` environment has deployment branch restrictions, allow `main`.
After merging, the workflow publishes to <https://picknikrobotics.github.io/generate_parameter_library/>. A manual run on `main` can retry a deployment after the Pages setting is configured.

The build job has read access to repository contents. Only the deployment job receives `pages: write` and `id-token: write` permissions, and it uses the `github-pages` environment. No personal access token or deployment branch is required.
