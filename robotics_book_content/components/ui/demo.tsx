'use client'

import { SplineScene } from "./splite";
import { Card } from "./card"
import { Spotlight } from "./spotlight"
import Link from '@docusaurus/Link'; // Import Docusaurus Link

interface SplineSceneBasicProps {
  title: string;
  subtitle: string;
  ctaLabel: string;
  ctaLink: string;
}
 
export function SplineSceneBasic({ title, subtitle, ctaLabel, ctaLink }: SplineSceneBasicProps) {
  return (
    <Card className="w-full h-[500px] bg-black/[0.96] relative overflow-hidden">
      <Spotlight
        className="-top-40 left-0 md:left-60 md:-top-20"
        fill="white"
      />
      
      <div className="flex h-full">
        {/* Left content */}
        <div className="flex-1 p-8 relative z-10 flex flex-col justify-center items-center text-center">
          <h1 className="text-4xl md:text-5xl font-bold bg-clip-text text-transparent bg-gradient-to-b from-neutral-50 to-neutral-400">
            {title}
          </h1>
          <p className="mt-4 text-neutral-300 max-w-lg">
            {subtitle}
          </p>
          <div className="mt-8">
            <Link
              className="button button--primary button--lg"
              to={ctaLink}>
              {ctaLabel}
            </Link>
          </div>
        </div>

        {/* Right content */}
        <div className="flex-1 relative">
          <SplineScene
            scene="https://prod.spline.design/kZDDjO5HuC9GJUM2/scene.splinecode"
            className="w-full h-full"
          />        </div>
      </div>
    </Card>
  )
}