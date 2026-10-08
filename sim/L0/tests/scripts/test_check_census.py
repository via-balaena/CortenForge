#!/usr/bin/env python3
"""Self-test: check_census_append_only.sh and check_census_verdicts.py refuse each
hand edit they exist to refuse, and pass the edits a commit may make.

    python3 sim/L0/tests/scripts/test_check_census.py

Each case builds a small census in a throwaway git repository (needs git and
bash), commits its edits on a branch from the base, and runs the check against
the base. A refused case names the words its error must contain. Set
CHECK_CENSUS to a path to test another copy of the script (it runs the
check_census_verdicts.py beside it).
"""
import os
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.abspath(os.environ.get("CHECK_CENSUS", os.path.join(HERE, "check_census_append_only.sh")))
CENSUS = os.path.join("sim", "L0", "tests", "assets", "census")
A, B, C, D, E, N = "a" * 16, "b" * 16, "c" * 16, "d" * 16, "f" * 16, "e" * 16

# The base census: two agree rows, a lowered row a later commit fixes, a refused
# row with a known= note, a row the census skips (nondet=).
BASE_ROWS = {
    A: ("agree", ""),
    B: ("agree", ""),
    C: ("e1:state@1", "fixed_by=P09 was=agree"),
    D: ("ours-refused", "known=x"),
    E: ("e1:state@1", "nondet=x"),
}
BASE_FLOOR = 2


class Census:
    """The work tree's census files, written whole on every edit."""

    def __init__(self, root):
        self.dir = os.path.join(root, CENSUS)
        self.rows = dict(BASE_ROWS)
        self.floor = BASE_FLOOR

    def write(self, rel, text):
        path = os.path.join(self.dir, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(text)

    def remove(self, rel):
        os.remove(os.path.join(self.dir, rel))

    def add_doc(self, doc, manifest=True):
        self.write(f"docs/{doc}.xml", f"<mujoco model=\"{doc}\"/>\n")
        self.write(f"golden/{doc}.json", '{"status":"ok"}\n')
        if manifest:
            self.append_manifest(doc)

    def append_manifest(self, doc):
        with open(os.path.join(self.dir, "manifest.tsv"), "a") as f:
            f.write(f"{doc}\tsrc/lib.rs:1\n")

    def write_verdicts(self):
        lines = ["# verdicts", f"# agree_floor {self.floor}"]
        for doc, (verdict, note) in sorted(self.rows.items()):
            lines.append("\t".join([doc, verdict] + ([note] if note else [])))
        self.write("verdicts.tsv", "\n".join(lines) + "\n")

    def set_row(self, doc, verdict, note=""):
        self.rows[doc] = (verdict, note)
        self.write_verdicts()

    def drop_row(self, doc):
        del self.rows[doc]
        self.write_verdicts()

    def set_floor(self, floor):
        self.floor = floor
        self.write_verdicts()

    def append_verdicts_line(self, line):
        with open(os.path.join(self.dir, "verdicts.tsv"), "a") as f:
            f.write(line + "\n")


class Repo:
    def __init__(self, root):
        self.root = root
        self.env = dict(os.environ, GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM="1",
                        GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@t",
                        GIT_COMMITTER_NAME="t", GIT_COMMITTER_EMAIL="t@t")

    def git(self, *args):
        subprocess.run(["git", "-c", "commit.gpgsign=false", *args], cwd=self.root, env=self.env,
                       check=True, capture_output=True)

    def commit(self, message):
        self.git("add", "-A")
        self.git("commit", "-q", "--allow-empty", "-m", message)

    def check(self):
        return subprocess.run(["bash", SCRIPT, "base"], cwd=self.root, env=self.env,
                              capture_output=True, text=True, timeout=120)


def base_census(census):
    census.write("golden/meta.json", "{}\n")
    census.write("manifest.tsv", "# manifest\n")
    census.write("divergences.tsv", "# registry\nID\tarea\nD-X\tx\n")
    for doc in BASE_ROWS:
        census.add_doc(doc)
    census.write_verdicts()


# Each case: (name, commits, refused-with). A commit is a function of the
# census; refused-with is None for an edit the check passes.
CASES = [
    ("no edit", [lambda c: None], None),
    ("add a doc with its golden, manifest line and row",
     [lambda c: (c.add_doc(N), c.set_row(N, "agree"), c.set_floor(3))], None),
    ("edit a golden", [lambda c: c.write(f"golden/{A}.json", '{"status":"ok" }\n')], "append-only"),
    ("delete a doc", [lambda c: (c.remove(f"docs/{A}.xml"), c.remove(f"golden/{A}.json"))],
     "append-only"),
    ("rename a doc", [lambda c: (c.remove(f"docs/{A}.xml"), c.write(f"docs/{N}.xml", "<mujoco/>\n"))],
     "append-only"),
    ("remove a manifest line", [lambda c: c.write("manifest.tsv", "# manifest\n" + "".join(
        f"{d}\tsrc/lib.rs:1\n" for d in (A, B, C, D)))], "only gains lines"),
    ("edit a manifest line", [lambda c: c.write("manifest.tsv", "# manifest\n" + "".join(
        f"{d}\tsrc/lib.rs:{2 if d == A else 1}\n" for d in BASE_ROWS))], "only gains lines"),
    ("edit a golden an earlier commit of the branch added",
     [lambda c: (c.add_doc(N), c.set_row(N, "agree"), c.set_floor(3)),
      lambda c: c.write(f"golden/{N}.json", '{"status":"ok" }\n')], "append-only"),
    ("lower a row with no note", [lambda c: (c.set_row(A, "e1:state@1"), c.set_floor(1))],
     "lowered from agree"),
    ("lower a row with a fixed_by note and the floor with it",
     [lambda c: (c.set_row(A, "e1:state@1", "fixed_by=P13 was=agree"), c.set_floor(1))], None),
    ("lower a row with a was= below its class",
     [lambda c: (c.set_row(A, "e1:state@1", "fixed_by=P13 was=dyn"), c.set_floor(1))],
     "lowered from agree"),
    ("lower a row with a malformed fixed_by note",
     [lambda c: (c.set_row(A, "e1:state@1", "fixed_by=X13 was=agree"), c.set_floor(1))],
     "lowered from agree"),
    ("add a nondet= note", [lambda c: c.set_row(B, "agree", "nondet=x")], "gained a nondet= note"),
    ("remove a row", [lambda c: (c.drop_row(B), c.set_floor(1))], "lost its row"),
    ("drop a fixed_by note from a row still below its was=", [lambda c: c.set_row(C, "e1:state@1")],
     "dropped or weakened its note"),
    ("lower a fixed_by note's was=", [lambda c: c.set_row(C, "e1:state@1", "fixed_by=P09 was=dyn")],
     "dropped or weakened its note"),
    ("raise a row part way and drop its fixed_by note",
     [lambda c: c.set_row(C, "model:geom_quat;dyn:agree")], "dropped or weakened its note"),
    ("move a fixed_by note to a later commit",
     [lambda c: c.set_row(C, "e1:state@1", "fixed_by=P13 was=agree")], None),
    ("clear a fixed_by note once the row is back to its was=",
     [lambda c: (c.set_row(C, "agree"), c.set_floor(3))], None),
    ("drop a known= note from a row that has not risen", [lambda c: c.set_row(D, "ours-refused")],
     "dropped or weakened its note"),
    ("turn a known= note into a divergence= note",
     [lambda c: c.set_row(D, "ours-refused", "divergence=D-X")], None),
    ("drop a known= note a fixed row kept, in a later commit",
     [lambda c: (c.set_row(D, "agree", "known=x"), c.set_floor(3)),
      lambda c: c.set_row(D, "agree")], None),
    ("lower a row with an empty known= note",
     [lambda c: (c.set_row(A, "e1:state@1", "known="), c.set_floor(1))], "lowered from agree"),
    ("drop a nondet= note", [lambda c: c.set_row(E, "e1:state@1")], None),
    ("lower the floor in one commit and restore the row in the next",
     [lambda c: (c.set_row(A, "e1:state@1", "fixed_by=P20 was=agree"), c.set_floor(1)),
      lambda c: c.set_row(A, "agree")], "agree_floor fell"),
    ("add a second floor line below the first",
     [lambda c: c.append_verdicts_line("# agree_floor 0")], "agree_floor fell"),
    ("lower the floor with no agree row lowered", [lambda c: c.set_floor(1)], "agree_floor fell"),
    ("lower the floor past the agree rows lowered",
     [lambda c: (c.set_row(A, "e1:state@1", "fixed_by=P13 was=agree"), c.set_floor(0))],
     "agree_floor fell"),
    ("a doc with no manifest line",
     [lambda c: (c.add_doc(N, manifest=False), c.set_row(N, "agree"), c.set_floor(3))],
     "no manifest.tsv line"),
    ("a manifest line with no doc", [lambda c: c.append_manifest(N)], "names no doc"),
]


class Check(unittest.TestCase):
    def test_cases(self):
        with tempfile.TemporaryDirectory() as root:
            repo = Repo(root)
            repo.git("init", "-q", "-b", "base")
            base_census(Census(root))
            repo.commit("base")
            for name, commits, refused_with in CASES:
                with self.subTest(case=name):
                    repo.git("checkout", "-q", "-f", "-B", "case", "base")
                    repo.git("clean", "-q", "-fdx")
                    census = Census(root)
                    for i, edit in enumerate(commits):
                        edit(census)
                        repo.commit(f"{name} {i}")
                    r = repo.check()
                    out = r.stdout + r.stderr
                    if refused_with is None:
                        self.assertEqual(r.returncode, 0, out)
                    else:
                        self.assertEqual(r.returncode, 1, out)
                        self.assertIn(refused_with, out)


if __name__ == "__main__":
    unittest.main()
