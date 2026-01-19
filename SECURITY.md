# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| 0.7.x   | :white_check_mark: |
| < 0.7   | :x:                |

## Reporting a Vulnerability

**Do not report security vulnerabilities through public GitHub issues.**

Instead, please report them via email to: security@cortenforge.dev

You should receive a response within 48 hours. If for some reason you do not, please follow up via email to ensure we received your original message.

Please include the following information:

- Type of issue (e.g., buffer overflow, SQL injection, cross-site scripting, etc.)
- Full paths of source file(s) related to the manifestation of the issue
- The location of the affected source code (tag/branch/commit or direct URL)
- Any special configuration required to reproduce the issue
- Step-by-step instructions to reproduce the issue
- Proof-of-concept or exploit code (if possible)
- Impact of the issue, including how an attacker might exploit the issue

## Security Practices

### Signed Commits

All commits to protected branches must be signed. This ensures:
- Commit authorship is verified (non-repudiation)
- Supply chain integrity (SLSA Level 2+)
- Defense against commit spoofing attacks

#### Setting Up Commit Signing

**Option 1: GPG Key**

```bash
# Generate a GPG key
gpg --full-generate-key

# Get your key ID
gpg --list-secret-keys --keyid-format LONG

# Configure git
git config --global user.signingkey YOUR_KEY_ID
git config --global commit.gpgsign true

# Add your public key to GitHub
gpg --armor --export YOUR_KEY_ID
# Paste output at: GitHub → Settings → SSH and GPG keys
```

**Option 2: SSH Key (Simpler)**

```bash
# Configure git to use SSH for signing
git config --global gpg.format ssh
git config --global user.signingkey ~/.ssh/id_ed25519.pub
git config --global commit.gpgsign true

# Add your SSH key to GitHub for signing
# GitHub → Settings → SSH and GPG keys → New SSH key
# Select "Signing Key" as the key type
```

#### Verifying Commits

```bash
# Verify a commit signature
git verify-commit HEAD

# Show signature in log
git log --show-signature -1
```

### Dependency Security

We use multiple layers of dependency security:

1. **cargo-audit**: Checks dependencies against RustSec advisory database
2. **cargo-deny**: Enforces license compliance and bans problematic crates
3. **Dependabot**: Automated security updates
4. **SBOM**: Every release includes a Software Bill of Materials

### Supply Chain Security

- All dependencies must come from crates.io or approved GitHub organizations
- No wildcard version specifications
- SBOM (CycloneDX format) generated for every release
- Weekly security scans via scheduled CI

### Code Safety

- Zero `unwrap()`/`expect()` in library code
- Zero `panic!()`/`todo!()`/`unimplemented!()` in shipped code
- All `unsafe` blocks require `// SAFETY:` documentation
- Static analysis via Clippy with pedantic lints

## Security Advisories

Security advisories will be published to:
- [GitHub Security Advisories](https://github.com/cortenforge/forge/security/advisories)
- The RustSec Advisory Database (for published crates)

## Disclosure Policy

- Security issues are fixed with highest priority
- Patches are released as soon as a fix is available
- Public disclosure occurs after patch is available
- Credit is given to reporters (unless anonymity is requested)

## Contact

- Security issues: security@cortenforge.dev
- General issues: [GitHub Issues](https://github.com/cortenforge/forge/issues)
