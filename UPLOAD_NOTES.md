# Workspace source snapshot

This upload includes the current local source files, including modifications inside nested repositories. Nested Git metadata is omitted, so the source is available directly after cloning. Original repository URLs and checked-out commits are recorded in `SOURCE_REPOSITORIES.json`; those commits do not include local modifications present in this snapshot.

Excluded: build/install output, logs, runtime evidence and recordings, backup folders, Python environments and caches, local agent/editor settings, database maps, downloaded archives, and the prebuilt `librealsense_251` installation. Runtime reports and handoff Markdown documents remain included; their references to excluded evidence refer to files retained on the original machine.

This is a source backup, not a verified portable deployment. See package documentation for dependencies.
