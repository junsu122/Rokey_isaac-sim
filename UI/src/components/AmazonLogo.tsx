export function AmazonLogo() {
  return (
    <svg width="140" height="44" viewBox="0 0 140 44" fill="none" xmlns="http://www.w3.org/2000/svg">
      {/* amaezon 텍스트 */}
      <text
        x="0" y="28"
        fontFamily="'Arial Black', 'Arial Bold', sans-serif"
        fontWeight="900"
        fontSize="26"
        fill="#FFFFFF"
        letterSpacing="-0.5"
      >
        amaezon
      </text>
      {/* 스마일 화살표 — a에서 n까지 */}
      <path
        d="M 8 36 Q 68 52 128 36"
        stroke="#FF9900"
        strokeWidth="3"
        strokeLinecap="round"
        fill="none"
      />
      {/* 화살표 끝 */}
      <path
        d="M 122 33 L 128 36 L 121 39"
        stroke="#FF9900"
        strokeWidth="2.5"
        strokeLinecap="round"
        strokeLinejoin="round"
        fill="none"
      />
    </svg>
  );
}
