## Summary

<!-- Brief description of what this PR does -->

## Type of Change

- [ ] `feat` - New feature
- [ ] `fix` - Bug fix
- [ ] `refactor` - Code refactoring
- [ ] `docs` - Documentation
- [ ] `test` - Tests
- [ ] `chore` - Maintenance

## Checklist

### Before Review

- [ ] Commit messages follow [conventional commits](https://www.conventionalcommits.org/)
- [ ] All commits are signed (verified badge)
- [ ] `cargo xtask check` passes locally
- [ ] New code has tests
- [ ] Documentation updated (if applicable)

### Quality Gates (CI will verify)

- [ ] Format: `cargo fmt --check`
- [ ] Lint: `cargo clippy -- -D warnings`
- [ ] Tests: `cargo test`
- [ ] Coverage: ≥90%
- [ ] Safety: No `unwrap()`/`expect()` in library code
- [ ] Docs: No warnings

## Related Issues

<!-- Link any related issues: Fixes #123, Relates to #456 -->

## Testing

<!-- How was this tested? -->

---

> **Reminder**: A-grade or it doesn't ship. No exceptions.
