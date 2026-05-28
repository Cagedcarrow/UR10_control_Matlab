/* ============================================================
   UR10 技术文档 — 共享 JS (导航 / 目录 / 滚动观察)
   ============================================================ */

(function() {
  'use strict';

  // --- Sidebar toggle for mobile ---
  const sidebar = document.getElementById('sidebar');
  const toggleBtn = document.getElementById('sidebarToggle');
  if (toggleBtn && sidebar) {
    toggleBtn.addEventListener('click', function(e) {
      e.stopPropagation();
      sidebar.classList.toggle('open');
    });
    document.addEventListener('click', function(e) {
      if (!sidebar.contains(e.target) && e.target !== toggleBtn) {
        sidebar.classList.remove('open');
      }
    });
  }

  // --- Auto-generate TOC ---
  const tocContainer = document.getElementById('toc');
  const mainContent = document.querySelector('.main-content');
  if (tocContainer && mainContent) {
    const headings = mainContent.querySelectorAll('h2, h3');
    if (headings.length > 1) {
      const tocTitle = document.createElement('h4');
      tocTitle.textContent = '目录';
      tocContainer.appendChild(tocTitle);

      headings.forEach(function(h, i) {
        const id = h.id || ('section-' + i);
        h.id = id;
        const link = document.createElement('a');
        link.href = '#' + id;
        link.textContent = h.textContent;
        link.className = h.tagName === 'H3' ? 'toc-h3' : '';
        tocContainer.appendChild(link);
      });

      // Scroll spy
      const observer = new IntersectionObserver(function(entries) {
        entries.forEach(function(entry) {
          const id = entry.target.id;
          const tocLink = tocContainer.querySelector('a[href="#' + id + '"]');
          if (tocLink) {
            if (entry.isIntersecting) {
              tocContainer.querySelectorAll('a.visible').forEach(function(a) { a.classList.remove('visible'); });
              tocLink.classList.add('visible');
            }
          }
        });
      }, { rootMargin: '-80px 0px -70% 0px', threshold: 0 });

      headings.forEach(function(h) { observer.observe(h); });
    }
  }

  // --- Highlight current nav item ---
  const currentPage = window.location.pathname.split('/').filter(Boolean).pop() || '';
  const navLinks = document.querySelectorAll('.nav-list li a');
  navLinks.forEach(function(a) {
    const href = a.getAttribute('href') || '';
    if (href.includes(currentPage) && currentPage !== '') {
      a.parentElement.classList.add('active');
    }
  });

  // --- Open external links in new tab ---
  document.querySelectorAll('a[href^="http"]').forEach(function(a) {
    a.setAttribute('target', '_blank');
    a.setAttribute('rel', 'noopener noreferrer');
  });

})();
