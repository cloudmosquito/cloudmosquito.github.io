/* ==========================================================================
   图注
   --------------------------------------------------------------------------
   不用在 Markdown 里写任何 HTML，图注直接取图片的属性：

     ![这是一句图注](图片.png)            → 用 alt
     ![](图片.png "这是一句图注")          → 用 title（优先）
     ![image-3](图片.png)                 → 看起来像文件名，忽略，不生成图注

   只有「整个段落里就只有这一张图」时才会包装成 <figure> + <figcaption>，
   所以正文里夹着的行内小图标、徽章之类不会被误伤。
   ========================================================================== */

document$.subscribe(({ body }) => {
  body.querySelectorAll(".md-typeset p").forEach((p) => {
    // 已经处理过（例如脚本被重复执行）
    if (p.closest("figure")) return;

    const images = p.querySelectorAll("img");
    if (images.length !== 1) return;

    // 段落里除了图片还有文字，说明是行内配图，跳过
    if (p.textContent.trim() !== "") return;

    const img = images[0];
    const title = (img.getAttribute("title") || "").trim();
    const alt = (img.getAttribute("alt") || "").trim();

    // alt 常被自动填成文件名或 image-1 / Figure_1.png 这类占位，不当图注用
    const looksLikeFile = /\.(png|jpe?g|gif|webp|svg|avif)$/i.test(alt);
    const looksLikePlaceholder =
      /^(image|img|figure|fig|screenshot|photo|picture|图片|截图|图像)[\s_.-]*\d*$/i.test(alt);
    const caption = title || (looksLikeFile || looksLikePlaceholder ? "" : alt);
    if (!caption) return;

    const figure = document.createElement("figure");
    figure.className = "md-figure";

    while (p.firstChild) figure.appendChild(p.firstChild);
    // 已经拿去当图注了，就别再让浏览器弹一个重复的 tooltip
    if (title) img.removeAttribute("title");

    const figcaption = document.createElement("figcaption");
    figcaption.textContent = caption;
    figure.appendChild(figcaption);

    p.replaceWith(figure);
  });
});
