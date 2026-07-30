#!/usr/bin/env python3
"""
Obsidian 笔记 → Zensical 网站 Markdown 格式转换工具

将 Obsidian 格式的笔记（Wiki-Link、Callout、`![[图片]]` 等）
转换为 Zensical/MkDocs 兼容的 Markdown 格式。

用法:
    python obsidian_to_zensical.py -i "控制理论/最优控制/优化理论速成.md"
    python obsidian_to_zensical.py --batch
    python obsidian_to_zensical.py --batch --dry-run
"""

import argparse
import json
import logging
import os
import re
import shutil
import sys
import urllib.parse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ─── 配置 ───────────────────────────────────────────────────

# 脚本所在目录，用于自动检测路径
_SCRIPT_DIR = Path(__file__).parent.resolve()  # tools/
_PROJECT_ROOT = _SCRIPT_DIR.parent             # cloudmosquito_site/
_DEFAULT_WEBSITE_DOCS = _PROJECT_ROOT / "docs"
_CONFIG_FILE = _SCRIPT_DIR / "convert_config.json"


def _load_config_file() -> Optional[dict]:
    """读取 convert_config.json（如果存在）。"""
    if _CONFIG_FILE.exists():
        try:
            with open(_CONFIG_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            log.warning("配置文件读取失败: %s", e)
    return None


@dataclass
class Config:
    """转换配置。website_docs 自动检测，notes_root 从配置文件或 CLI 读取。"""

    notes_root: Path = field(default_factory=lambda: Path("."))
    website_docs: Path = field(default_factory=lambda: _DEFAULT_WEBSITE_DOCS)

    obsidian_imgs_dir: str = "obsidian_imgs"
    assets_suffix: str = ".assets"
    default_img_width: str = "50%"

    # Obsidian callout 类型 → Zensical 输出类型（小写）
    admonition_types: dict = field(default_factory=lambda: {
        "note": "note", "warning": "warning", "tip": "tip",
        "info": "info", "question": "question", "example": "example",
        "quote": "quote", "theorem": "theorem", "abstract": "abstract",
        "success": "success", "failure": "failure", "danger": "danger",
        "bug": "bug", "todo": "todo", "hint": "hint",
        "comment": "comment",
    })

    # 不需要转换的目录
    skip_dirs: tuple = (".trash", ".obsidian", ".git", "obsidian_imgs", ".assets")


# ─── 日志 ───────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(levelname)s: %(message)s",
)
log = logging.getLogger(__name__)


# ─── 辅助函数 ────────────────────────────────────────────────


def get_code_block_ranges(lines: list[str]) -> list[tuple[int, int]]:
    """返回代码块内部行范围（不含 fence 自身），0-indexed，闭区间。"""
    ranges = []
    in_block = False
    start = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            if not in_block:
                in_block = True
                start = i + 1  # 内容从 fence 下一行开始
            else:
                if start <= i - 1:
                    ranges.append((start, i - 1))  # 内容到 fence 前一行结束
                in_block = False
    if in_block:
        ranges.append((start, len(lines) - 1))
    return ranges


def get_math_block_ranges(lines: list[str]) -> list[tuple[int, int]]:
    """返回 $$...$$ 数学块内部行范围（不含分隔符），0-indexed，闭区间。
    单行 $$...$$ 不产生保护范围（整行即是内容又是分隔符）。
    """
    ranges = []
    in_math = False
    start = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        starts_with_dd = stripped.startswith("$$")
        ends_with_dd = stripped.endswith("$$")

        if not in_math:
            if starts_with_dd:
                if ends_with_dd and len(stripped) > 2:
                    # 单行: $$...$$ — 不保护（就一行，不会被段落间距破坏）
                    continue
                else:
                    # 多行开始，内容从下一行开始
                    in_math = True
                    start = i + 1
        else:
            if ends_with_dd:
                if start <= i - 1:
                    ranges.append((start, i - 1))  # 内容到 $$ 前一行结束
                in_math = False

    if in_math:
        ranges.append((start, len(lines) - 1))
    return ranges


def get_protected_ranges(lines: list[str]) -> list[tuple[int, int]]:
    """返回所有保护区（代码块 + 数学块）。"""
    code = get_code_block_ranges(lines)
    math = get_math_block_ranges(lines)
    return sorted(code + math, key=lambda x: x[0])


def is_in_code_block(line_idx: int, code_ranges: list[tuple[int, int]]) -> bool:
    """判断某行是否在代码块内。"""
    return any(start <= line_idx <= end for start, end in code_ranges)


def is_protected(line_idx: int, protected_ranges: list[tuple[int, int]]) -> bool:
    """判断某行是否在保护区（代码块或数学块）内。"""
    return any(start <= line_idx <= end for start, end in protected_ranges)


def url_encode_path(path: str) -> str:
    """URL 编码路径中的中文和特殊字符，保留路径分隔符和常见安全字符。"""
    # 分别编码每个路径段
    parts = path.replace("\\", "/").split("/")
    encoded = []
    for p in parts:
        if p in (".", ".."):
            encoded.append(p)
        else:
            encoded.append(urllib.parse.quote(p, safe=".-_"))
    return "/".join(encoded)


def compute_relative_path(from_file: Path, to_file: Path) -> str:
    """计算 from_file 到 to_file 的相对路径（含 .md 扩展名）。"""
    try:
        from_dir = str(Path(from_file).parent.resolve())
        to_path = str(Path(to_file).resolve())
        rel = os.path.relpath(to_path, from_dir)
        return rel.replace("\\", "/")
    except ValueError:
        return str(to_file.name)


def find_target_page(page_name: str, notes_root: Path) -> Optional[Path]:
    """在笔记目录树中搜索目标 .md 文件。"""
    # 清理页面名
    clean = page_name.strip()
    target_name = clean if clean.endswith(".md") else clean + ".md"

    for md_file in notes_root.rglob("*.md"):
        # 跳过垃圾目录
        parts = md_file.parts
        if any(skip in parts for skip in [".trash", ".obsidian", "obsidian_imgs"]):
            continue
        if md_file.name == target_name:
            return md_file
        # 大小写不敏感匹配
        if md_file.name.lower() == target_name.lower():
            return md_file

    return None


def find_image_file(image_name: str, notes_root: Path, source_dir: Path) -> Optional[Path]:
    """搜索图片文件。优先从 obsidian_imgs/ 查找，其次在同目录查找。"""
    # 1. obsidian_imgs 目录
    imgs_dir = notes_root / "obsidian_imgs"
    if imgs_dir.exists():
        candidate = imgs_dir / image_name
        if candidate.exists():
            return candidate

    # 2. 同目录
    candidate = source_dir / image_name
    if candidate.exists():
        return candidate

    # 3. 递归搜索（排除 .trash 等）
    for f in notes_root.rglob(image_name):
        parts = f.parts
        if any(skip in parts for skip in [".trash", ".obsidian"]):
            continue
        return f

    return None


# ─── Pass 1: 段落间距规范化 ─────────────────────────────────


def normalize_paragraph_spacing(text: str) -> str:
    """确保段落、列表、代码块等元素之间有空行。"""
    lines = text.split("\n")
    protected = get_protected_ranges(lines)
    result = []
    i = 0
    while i < len(lines):
        result.append(lines[i])

        if i + 1 >= len(lines):
            i += 1
            continue

        cur = lines[i].rstrip()
        nxt = lines[i + 1].rstrip()
        nxt_is_blank = (nxt == "")

        # 跳过已有空行的情况
        if cur == "" or nxt_is_blank:
            i += 1
            continue

        # 跳过保护区
        if is_protected(i, protected) or is_protected(i + 1, protected):
            i += 1
            continue

        need_blank = False

        # 当前行是"内容"（普通文本/段落，非列表、非代码块、非标题等）
        cur_is_list = bool(re.match(r'^(\d+\.\s|[-*+]\s)', cur))
        cur_is_content = bool(cur) and not cur.startswith("#") \
            and not cur.startswith("```") and not cur.startswith("~~~") \
            and not cur.startswith("!!!") and not cur.startswith("???") \
            and not cur.startswith("$$") and cur != "---" \
            and not cur.startswith("|") and not cur_is_list

        # 下一行是列表项（-  *  +  或 1. 等编号）
        nxt_is_list = bool(re.match(r'^(\d+\.\s|[-*+]\s)', nxt))

        # 下一行是代码块开头
        nxt_is_code_fence = nxt.startswith("```") or nxt.startswith("~~~")

        # 下一行是普通文本段落
        nxt_is_text = bool(nxt) and not nxt.startswith("#") \
            and not nxt.startswith(">") \
            and not nxt.startswith("- ") and not nxt.startswith("* ") \
            and not nxt.startswith("|") and nxt != "---" \
            and not nxt.startswith("```") and not nxt.startswith("~~~") \
            and not nxt.startswith("!!!") and not nxt.startswith("???") \
            and not nxt.startswith("$$") \
            and not nxt_is_list

        if cur_is_content:
            # 内容后接文本段落 → 空行
            if nxt_is_text:
                need_blank = True
            # 内容后接列表第一项 → 空行
            elif nxt_is_list:
                need_blank = True
            # 内容后接代码块 → 空行
            elif nxt_is_code_fence:
                need_blank = True

        if need_blank:
            result.append("")

        i += 1

    return "\n".join(result)


# ─── Pass 2: Admonition 转换 ─────────────────────────────────


def convert_admonitions(text: str) -> str:
    r"""> [!TYPE] Title ... > body → !!! Type "Title"\n\n    body"""
    lines = text.split("\n")
    code_ranges = get_code_block_ranges(lines)
    math_ranges = get_math_block_ranges(lines)
    protected = sorted(code_ranges + math_ranges, key=lambda x: x[0])
    result = []
    i = 0
    while i < len(lines):
        line = lines[i]

        if is_in_code_block(i, code_ranges):
            result.append(line)
            i += 1
            continue

        # 检测 Obsidian callout 开头: > [!TYPE] 或 > [!TYPE] 标题
        m = re.match(r'^>\s*\[!(\w+)\]\s*(.*?)$', line, re.IGNORECASE)
        if m:
            obs_type = m.group(1).lower()
            obs_title = m.group(2).strip()

            # 映射到 Zensical 类型
            zensical_type = Config().admonition_types.get(
                obs_type, obs_type.lower()
            )

            # 收集 body 行（所有以 > 开头的连续行）
            body_lines = []
            j = i + 1
            while j < len(lines):
                body_line = lines[j]
                if body_line.startswith(">"):
                    # 去掉 > 前缀，如果后面有空格也去掉
                    content = body_line[1:]
                    if content.startswith(" "):
                        content = content[1:]
                    body_lines.append(content)
                    j += 1
                elif body_line.strip() == "":
                    # 真正的空行（无 > 前缀）：admonition 结束
                    break
                else:
                    break

            i = j  # 跳到 body 之后

            # 去掉 body 开头和结尾的多余空行
            while body_lines and body_lines[0] == "":
                body_lines.pop(0)
            while body_lines and body_lines[-1] == "":
                body_lines.pop()

            # 构建 Zensical admonition
            if obs_title:
                header = f'!!! {zensical_type} "{obs_title}"'
            else:
                header = f"!!! {zensical_type}"

            result.append(header)
            result.append("")  # 空行

            for bl in body_lines:
                if bl == "" or bl.strip() == "":
                    result.append("")
                else:
                    result.append(f"    {bl}")

            # admonition 后加空行（如果下一行非空）
            if i < len(lines) and lines[i].strip() != "":
                result.append("")

            continue

        result.append(line)
        i += 1

    return "\n".join(result)


# ─── Pass 3: Wiki-Link 转换 ──────────────────────────────────


def convert_wiki_links(text: str, source_path: Path, config: Config) -> str:
    """[[Page]] / [[Page|Alias]] / [[Page#Section]] → markdown 链接。"""
    lines = text.split("\n")
    code_ranges = get_code_block_ranges(lines)
    math_ranges = get_math_block_ranges(lines)
    protected = sorted(code_ranges + math_ranges, key=lambda x: x[0])
    result = []
    changes = 0

    for i, line in enumerate(lines):
        if is_in_code_block(i, code_ranges):
            result.append(line)
            continue

        # 跳过已经是 markdown 链接的行（包含 ]( 模式，但不在代码块中）
        # 仅处理 [[...]] 格式

        def replace_wiki_link(m):
            nonlocal changes
            inner = m.group(1)  # [[inner]]
            alias = m.group(2) if m.lastindex and m.group(2) else None  # |alias

            # 解析 # 锚点
            anchor = ""
            page_part = inner
            if "#" in inner and not inner.startswith("#"):
                page_part, anchor = inner.split("#", 1)
                anchor = "#" + anchor

            # 去掉可能的后缀
            page_name = page_part.strip()

            # 搜索目标文件
            target = find_target_page(page_name, config.notes_root)
            if target is None:
                log.warning("Wiki-Link 目标未找到: [[%s]] (来自 %s)", inner, source_path)
                return m.group(0)  # 保留原样

            # 计算相对路径
            try:
                from_dir = str(Path(source_path).parent.resolve())
                to_path = str(target.resolve())
                rel = os.path.relpath(to_path, from_dir)
                rel = rel.replace("\\", "/")
            except ValueError:
                rel = target.name

            # URL 编码路径
            encoded = url_encode_path(rel)
            display = alias if alias else page_name

            changes += 1
            return f"[{display}]({encoded}{anchor})"

        # 使用负向后顾 (?<!!) 避免匹配 ![[...]] 图片语法
        new_line = re.sub(
            r'(?<!!)\[\[([^\[\]]+?)(?:\|([^\[\]]*))?\]\]',
            replace_wiki_link,
            line
        )
        result.append(new_line)

    if changes > 0:
        log.info("  转换了 %d 个 Wiki-Link", changes)
    return "\n".join(result)


# ─── Pass 4: 图片引用转换 ───────────────────────────────────


def convert_image_refs(
    text: str, source_path: Path, config: Config, dry_run: bool = False
) -> str:
    """![[image.png|size]] → ![](./.assets/image.png){.img-center width=X%}"""
    lines = text.split("\n")
    code_ranges = get_code_block_ranges(lines)
    math_ranges = get_math_block_ranges(lines)
    protected = sorted(code_ranges + math_ranges, key=lambda x: x[0])
    result = []
    changes = 0
    copied = 0

    # 确定输出文件的基本名（不含扩展名）
    source_stem = Path(source_path).stem

    for i, line in enumerate(lines):
        if is_in_code_block(i, code_ranges):
            result.append(line)
            continue

        # 跳过已转换的 Zensical 图片
        if re.search(r'!\[.*?\]\(.*?\.assets/', line):
            result.append(line)
            continue

        def replace_image(m):
            nonlocal changes, copied
            img_name = m.group(1).strip()
            width_px = m.group(2)  # Obsidian 的 |宽度 语法

            # 清理图片名（去掉可能的路径）
            img_basename = Path(img_name).name

            # 搜索图片文件
            source_dir = Path(source_path).parent
            img_file = find_image_file(img_basename, config.notes_root, source_dir)

            if img_file is None:
                log.warning("图片未找到: %s (来自 %s)", img_basename, source_path)
                return m.group(0)  # 保留原样

            # 确定目标 .assets 文件夹
            output_dir = Path(source_path).parent
            assets_dir = output_dir / f"{source_stem}{config.assets_suffix}"

            # 复制图片
            if not dry_run:
                assets_dir.mkdir(parents=True, exist_ok=True)
                dest = assets_dir / img_file.name
                if not dest.exists():
                    shutil.copy2(img_file, dest)
                    copied += 1
            else:
                dest = assets_dir / img_file.name

            # 生成引用（图片名不需要 URL 编码）
            rel_path = f"./{source_stem}{config.assets_suffix}/{img_file.name}"

            # 宽度处理
            width_attr = f"width={config.default_img_width}"
            if width_px:
                # Obsidian 像素宽度 → 百分比（保持原有的大致比例）
                try:
                    px = int(width_px)
                    # 按比例估算百分比（假设图片典型宽度 800px）
                    pct = min(100, max(10, round(px / 8)))
                    width_attr = f"width={pct}%"
                except ValueError:
                    pass

            changes += 1
            return f"![]({rel_path}){{.img-center {width_attr}}}"

        new_line = re.sub(
            r'!\[\[([^\[\]|]+)(?:\|(\d+(?:x\d+)?))?\]\]',
            replace_image,
            line
        )
        result.append(new_line)

    if changes > 0:
        log.info("  转换了 %d 张图片%s", changes,
                 f"，复制了 {copied} 个文件" if copied > 0 else "")
    return "\n".join(result)


# ─── Pass 5: 数学公式间距修复 ─────────────────────────────────


def fix_math_block_spacing(text: str) -> str:
    """确保 $$...$$ 块前后各空一行（开头 $$ 前空行，结尾 $$ 后空行）。"""
    lines = text.split("\n")
    code_ranges = get_code_block_ranges(lines)

    # 使用和 get_math_block_ranges 相同的逻辑标记数学块
    in_math = False
    math_starts = set()
    math_ends = set()

    for i, line in enumerate(lines):
        if is_in_code_block(i, code_ranges):
            continue
        stripped = line.strip()
        starts_with_dd = stripped.startswith("$$")
        ends_with_dd = stripped.endswith("$$")

        if not in_math:
            if starts_with_dd:
                if ends_with_dd and len(stripped) > 2:
                    # 单行: $$...$$
                    math_starts.add(i)
                    math_ends.add(i)
                else:
                    in_math = True
                    math_starts.add(i)
        else:
            if ends_with_dd:
                math_ends.add(i)
                in_math = False

    # 在数学块前后加空行
    inserts = {}  # line_idx → 插入此索引之前

    for start_idx in sorted(math_starts):
        if start_idx > 0 and lines[start_idx - 1].strip() != "":
            inserts.setdefault(start_idx, []).insert(0, "")

    for end_idx in sorted(math_ends):
        if end_idx + 1 < len(lines) and lines[end_idx + 1].strip() != "":
            inserts.setdefault(end_idx + 1, []).append("")

    if not inserts:
        return text

    new_lines = []
    for i, line in enumerate(lines):
        if i in inserts:
            for ins in inserts[i]:
                new_lines.append(ins)
        new_lines.append(line)

    return "\n".join(new_lines)


# ─── Pass 6: H1 标题 ─────────────────────────────────────────


def ensure_h1_title(text: str, filename_stem: str) -> str:
    """如果第一行不是 # 标题，则添加 # 文件名。"""
    lines = text.split("\n")

    # 跳过开头的空行
    first_content = 0
    while first_content < len(lines) and lines[first_content].strip() == "":
        first_content += 1

    if first_content >= len(lines):
        return f"# {filename_stem}\n\n"

    first_line = lines[first_content].strip()
    if first_line.startswith("# "):
        return text  # 已有 H1

    # 添加 H1
    title = f"# {filename_stem}"
    # 在前面加标题，保留原有空行结构
    pre = lines[:first_content]
    post = lines[first_content:]
    return "\n".join(pre + [title, ""] + post)


# ─── Pass 7: .md 扩展名修复 ───────────────────────────────────


def fix_markdown_link_extensions(text: str, source_path: Path, config: Config) -> str:
    """补全 markdown 链接中缺失的 .md 扩展名。"""
    lines = text.split("\n")
    code_ranges = get_code_block_ranges(lines)
    math_ranges = get_math_block_ranges(lines)
    protected = sorted(code_ranges + math_ranges, key=lambda x: x[0])
    result = []

    for i, line in enumerate(lines):
        if is_in_code_block(i, code_ranges):
            result.append(line)
            continue

        def fix_ext(m):
            full = m.group(0)
            link_text = m.group(1)
            link_target = m.group(2)

            # 跳过外部链接和纯锚点
            if link_target.startswith("http://") or link_target.startswith("https://"):
                return full
            if link_target.startswith("#"):
                return full
            if link_target.endswith(".md"):
                return full
            # 跳过已编码的 assets 路径
            if ".assets/" in link_target:
                return full
            # 跳过带属性标记的
            if "{" in link_target:
                return full

            # 检查目标是否是一个存在的 .md 文件
            clean_target = link_target.split("#")[0]  # 去掉锚点
            anchor = ""
            if "#" in link_target:
                anchor = "#" + link_target.split("#", 1)[1]

            # 尝试在笔记目录中查找
            target_name = Path(clean_target).name
            if not target_name:
                return full

            target_file = find_target_page(target_name, config.notes_root)
            if target_file:
                # URL 编码
                encoded = url_encode_path(clean_target + ".md")
                return f"[{link_text}]({encoded}{anchor})"

            return full

        new_line = re.sub(
            r'\[([^\]]*?)\]\(((?!(?:https?:|#|javascript:))[^)\s]+?)\)',
            fix_ext,
            line
        )
        result.append(new_line)

    return "\n".join(result)


# ─── 修复重复的 {.img-center} 属性 ───────────────────────────


def fix_duplicate_styling(text: str) -> str:
    """合并重复的 {.img-center width=X%}{.img-center width=X%}"""
    # 模式: {.img-center width=N%}{.img-center width=M%}
    pattern = r'(\{\.img-center\s+width=\d+%\})\s*(\{\.img-center\s+width=\d+%\})'
    fixed = re.sub(pattern, r'\1', text)
    return fixed


# ─── 主转换函数 ──────────────────────────────────────────────


def convert_file(
    source_path: Path,
    config: Config,
    dry_run: bool = False,
    skip_images: bool = False,
) -> Optional[str]:
    """转换单个文件，返回转换后的文本。"""
    if not source_path.exists():
        log.error("源文件不存在: %s", source_path)
        return None

    log.info("处理: %s", source_path)

    text = source_path.read_text(encoding="utf-8")
    original = text

    # 计算相对于笔记根目录的路径
    try:
        rel_path = source_path.relative_to(config.notes_root)
    except ValueError:
        rel_path = Path(source_path.name)

    filename_stem = source_path.stem

    # Pass 0: 修复已有的重复属性
    text = fix_duplicate_styling(text)

    # Pass 1: 段落间距
    text = normalize_paragraph_spacing(text)

    # Pass 2: Admonition
    text = convert_admonitions(text)

    # Pass 3: Wiki-Link
    text = convert_wiki_links(text, source_path, config)

    # Pass 4: 图片引用
    if not skip_images:
        text = convert_image_refs(text, source_path, config, dry_run=dry_run)

    # Pass 5: 公式间距
    text = fix_math_block_spacing(text)

    # Pass 6: H1 标题
    text = ensure_h1_title(text, filename_stem)

    # Pass 7: .md 扩展名
    text = fix_markdown_link_extensions(text, source_path, config)

    return text


def get_output_path(source_path: Path, config: Config) -> Path:
    """根据源文件路径计算网站输出路径。"""
    try:
        rel = source_path.relative_to(config.notes_root)
    except ValueError:
        rel = Path(source_path.name)
    return config.website_docs / rel


# ─── CLI ─────────────────────────────────────────────────────


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Obsidian 笔记 → Zensical 网站 Markdown 格式转换",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s -i "控制理论/最优控制/优化理论速成.md"
  %(prog)s --batch
  %(prog)s --batch --dry-run
  %(prog)s --batch --only-modified
        """,
    )

    parser.add_argument(
        "-i", "--input", type=str,
        help="源笔记 .md 文件（单文件模式，相对于笔记根目录的路径）"
    )
    parser.add_argument(
        "-o", "--output", type=str,
        help="输出 .md 文件路径（单文件模式，默认自动推导）"
    )
    parser.add_argument(
        "--batch", action="store_true",
        help="批量转换笔记目录下所有文件"
    )
    parser.add_argument(
        "--dry-run", "-n", action="store_true",
        help="预览模式：只显示转换结果，不写入文件"
    )
    parser.add_argument(
        "--force", "-f", action="store_true",
        help="强制覆盖已有输出"
    )
    parser.add_argument(
        "--skip-images", action="store_true",
        help="跳过图片复制"
    )
    parser.add_argument(
        "--only-modified", action="store_true",
        help="仅转换比输出更新的源文件"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="详细输出"
    )
    parser.add_argument(
        "--notes-root", type=str,
        help="笔记根目录路径"
    )
    parser.add_argument(
        "--website-root", type=str,
        help="网站 docs 目录（默认: 自动检测为脚本 ../../docs/）"
    )
    parser.add_argument(
        "--init-config", type=str, metavar="NOTES_ROOT",
        help="创建配置文件 convert_config.json 并写入笔记根目录路径"
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.verbose:
        log.setLevel(logging.DEBUG)

    # ── 初始化配置文件 ──
    if args.init_config:
        notes_path = Path(args.init_config)
        if not notes_path.exists():
            log.error("路径不存在: %s", notes_path)
            sys.exit(1)
        cfg_data = {"notes_root": str(notes_path.resolve())}
        _CONFIG_FILE.write_text(json.dumps(cfg_data, ensure_ascii=False, indent=2),
                                encoding="utf-8")
        log.info("配置文件已创建: %s", _CONFIG_FILE)
        log.info("笔记根目录: %s", notes_path.resolve())
        return

    # ── 确定 notes_root ──
    cfg = _load_config_file()
    notes_root = None

    if args.notes_root:
        notes_root = Path(args.notes_root)
    elif cfg and "notes_root" in cfg:
        notes_root = Path(cfg["notes_root"])

    if notes_root is None:
        print("错误: 未指定笔记根目录。请通过以下方式之一指定：")
        print(f"  1. 命令行: --notes-root <路径>")
        print(f"  2. 配置文件: 在 {_CONFIG_FILE} 中写入 JSON：")
        print(f'     {{"notes_root": "E:/你的笔记目录路径"}}')
        print(f"\n是否需要现在创建配置文件？运行：")
        print(f'  python {__file__} --init-config "E:/你的笔记目录路径"')
        sys.exit(1)

    if not notes_root.exists():
        log.error("笔记根目录不存在: %s", notes_root)
        sys.exit(1)

    # ── 确定 website_docs ──
    website_docs = Path(args.website_root) if args.website_root else _DEFAULT_WEBSITE_DOCS

    config = Config(notes_root=notes_root, website_docs=website_docs)

    # ── 单文件模式 ──
    if args.input:
        source = config.notes_root / args.input
        if not source.exists():
            log.error("文件不存在: %s", source)
            sys.exit(1)

        output = Path(args.output) if args.output else get_output_path(source, config)

        text = convert_file(source, config, dry_run=args.dry_run,
                           skip_images=args.skip_images)
        if text is None:
            sys.exit(1)

        if args.dry_run:
            sys.stdout.buffer.write(text.encode('utf-8'))
            sys.stdout.buffer.write(b"\n")
        else:
            output.parent.mkdir(parents=True, exist_ok=True)
            if output.exists() and not args.force:
                log.warning("输出文件已存在，使用 --force 强制覆盖: %s", output)
                sys.exit(1)
            output.write_text(text, encoding="utf-8")
            log.info("已写入: %s", output)

        return

    # ── 批量模式 ──
    if args.batch:
        notes_root = config.notes_root
        total = 0
        written = 0
        skipped = 0

        for md_file in sorted(notes_root.rglob("*.md")):
            # 跳过特殊目录
            parts = md_file.parts
            if any(s in parts for s in config.skip_dirs):
                continue

            total += 1
            output_path = get_output_path(md_file, config)

            # 仅转换修改过的
            if args.only_modified and output_path.exists():
                if output_path.stat().st_mtime >= md_file.stat().st_mtime:
                    skipped += 1
                    continue

            if not args.dry_run and output_path.exists() and not args.force:
                log.warning("输出已存在，跳过: %s", output_path)
                skipped += 1
                continue

            text = convert_file(md_file, config, dry_run=args.dry_run,
                               skip_images=args.skip_images)
            if text is None:
                continue

            if args.dry_run:
                # 用 UTF-8 写出，避免 Windows GBK 终端编码问题
                sys.stdout.buffer.write(
                    f"\n{'='*60}\n文件: {md_file}\n{'='*60}\n".encode('utf-8')
                )
                sys.stdout.buffer.write(text.encode('utf-8'))
                sys.stdout.buffer.write(b"\n")
            else:
                output_path.parent.mkdir(parents=True, exist_ok=True)
                output_path.write_text(text, encoding="utf-8")
                written += 1

        log.info("完成: %d 个文件, 写入 %d, 跳过 %d", total, written, skipped)
        return

    # ── 无参数 ──
    print("请指定 --input 或 --batch。使用 -h 查看帮助。")
    sys.exit(1)


if __name__ == "__main__":
    main()
